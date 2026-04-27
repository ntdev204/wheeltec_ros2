"""
context_aware_bridge — ROS2 node bridging the Jetson AI server to wheeltec_ros2.

Wire protocol (read from context-aware/src/communication/zmq_publisher.py):

  Jetson → RasPi (ZMQ PUB :5555, topic b"ai/nav_cmd"):
      struct.pack("!iffiffB", mode, velocity_scale, heading_offset,
                              follow_target_id, timestamp, confidence,
                              safety_override)   -- 25 bytes, big-endian

  RasPi → Jetson (ZMQ PUB :5560, topic b"robot/state"):
      Protobuf RobotState (primary)
      Fallback accepted by Jetson: struct.pack("!7fd", vx, vy, vtheta,
                                                       pos_x, pos_y, pos_theta,
                                                       battery, timestamp) -- 36 bytes
"""

import math
import struct
import time
from threading import Thread

import rclpy
import zmq
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# NavigationMode enum values (mirrors context-aware/proto/messages.proto)
NAV_CRUISE   = 0
NAV_CAUTIOUS = 1
NAV_AVOID    = 2
NAV_FOLLOW   = 3
NAV_STOP     = 4

# Struct format for NavigationCommand (Jetson → RasPi), 25 bytes big-endian
# Fields: mode(i) vel_scale(f) heading_offset(f) follow_id(i) ts(f) confidence(f) safety_override(B)
_NAV_CMD_FMT  = '!iffiffB'
_NAV_CMD_SIZE = struct.calcsize(_NAV_CMD_FMT)  # 25

# Struct format for RobotState fallback (RasPi → Jetson), 36 bytes big-endian
# Fields: vx(f) vy(f) vtheta(f) pos_x(f) pos_y(f) pos_theta(f) battery(f) timestamp(d)
_ROBOT_STATE_FMT  = '!7fd'
_ROBOT_STATE_SIZE = struct.calcsize(_ROBOT_STATE_FMT)  # 36

# Velocity limits
MAX_LINEAR_VEL  = 0.8   # m/s — full velocity_scale=1.0 maps to this
MAX_ANGULAR_VEL = 1.5   # rad/s — ±π/4 heading_offset maps to this

# Watchdog: zero velocity if no nav_cmd received within this window
WATCHDOG_TIMEOUT_SEC = 1.5


class ContextAwareBridgeNode(Node):
    def __init__(self):
        super().__init__('context_aware_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('jetson_ip',        '25.12.4.100')
        self.declare_parameter('raspi_ip',         '25.12.4.101')
        self.declare_parameter('nav_cmd_port',     5555)
        self.declare_parameter('robot_state_port', 5560)

        jetson_ip        = self.get_parameter('jetson_ip').value
        robot_state_port = self.get_parameter('robot_state_port').value
        nav_cmd_port     = self.get_parameter('nav_cmd_port').value

        # ── ROS2 interfaces ─────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_context', 10)

        odom_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._odom_cb, odom_qos)
        self.create_subscription(Float32, '/PowerVoltage', self._voltage_cb, 10)

        # ── ZMQ setup ───────────────────────────────────────────────────────
        self._ctx = zmq.Context()

        # SUB: receive NavigationCommand from Jetson (struct.pack binary, 25 bytes)
        self._nav_sub = self._ctx.socket(zmq.SUB)
        self._nav_sub.setsockopt(zmq.RCVHWM, 2)
        self._nav_sub.setsockopt(zmq.LINGER, 0)
        self._nav_sub.setsockopt(zmq.RCVTIMEO, 200)
        self._nav_sub.setsockopt_string(zmq.SUBSCRIBE, 'ai/nav_cmd')
        self._nav_sub.connect(f'tcp://{jetson_ip}:{nav_cmd_port}')

        # PUB: publish RobotState to Jetson (Protobuf preferred, struct fallback)
        self._state_pub = self._ctx.socket(zmq.PUB)
        self._state_pub.setsockopt(zmq.SNDHWM, 5)
        self._state_pub.setsockopt(zmq.LINGER, 0)
        self._state_pub.bind(f'tcp://0.0.0.0:{robot_state_port}')

        # Try loading proto stubs generated inside Jetson Docker
        self._pb = None
        try:
            from src.communication.proto import messages_pb2 as pb
            self._pb = pb
            self.get_logger().info('Protobuf stubs loaded — RobotState will use Protobuf encoding.')
        except ImportError:
            self.get_logger().warn(
                'Protobuf stubs not found (src.communication.proto.messages_pb2). '
                'Falling back to struct encoding for RobotState. '
                'Jetson zmq_subscriber.py handles both formats.'
            )

        # ── Runtime state ────────────────────────────────────────────────────
        self._last_cmd_time = time.monotonic()
        self._odom: dict = {}
        self._battery_percent = 0.0  # Will be updated by /PowerVoltage callback

        # ── Threads & timers ─────────────────────────────────────────────────
        self._recv_thread = Thread(target=self._recv_loop, daemon=True, name='zmq-nav-sub')
        self._recv_thread.start()

        self.create_timer(0.1, self._watchdog_tick)   # 10 Hz watchdog

        self.get_logger().info(
            f'[context_aware_bridge] started\n'
            f'  SUB  tcp://{jetson_ip}:{nav_cmd_port}  (ai/nav_cmd)\n'
            f'  PUB  tcp://0.0.0.0:{robot_state_port}  (robot/state)'
        )

    # ── /odom callback → pack RobotState → ZMQ ──────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = {
            'vx':    msg.twist.twist.linear.x,
            'vy':    msg.twist.twist.linear.y,
            'vtheta': msg.twist.twist.angular.z,
            'pos_x':  msg.pose.pose.position.x,
            'pos_y':  msg.pose.pose.position.y,
            'pos_theta': self._yaw_from_quat(msg.pose.pose.orientation),
        }
        payload = self._encode_robot_state(self._odom)
        try:
            self._state_pub.send_multipart([b'robot/state', payload], flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    # ── /PowerVoltage callback (Volts → %) ─────────────────────────────────
    def _voltage_cb(self, msg: Float32) -> None:
        # Wheeltec 24V system (6S LiPo): Full=25.2V, Low=21.0V (safe threshold)
        voltage = msg.data
        if voltage > 25.2:
            self._battery_percent = 100.0
        elif voltage < 21.0:
            self._battery_percent = 0.0
        else:
            # Linear mapping: (V - 21.0) / (25.2 - 21.0) * 100
            self._battery_percent = (voltage - 21.0) / 4.2 * 100.0

    # ── ZMQ receive loop ─────────────────────────────────────────────────────
    def _recv_loop(self) -> None:
        while rclpy.ok():
            try:
                parts = self._nav_sub.recv_multipart()
                if len(parts) < 2:
                    continue
                raw = parts[1]
                if len(raw) != _NAV_CMD_SIZE:
                    self.get_logger().warn(
                        f'Unexpected nav_cmd size: {len(raw)} bytes (expected {_NAV_CMD_SIZE})'
                    )
                    continue
                mode, vel_scale, heading_offset, _, _, _, safety_override = \
                    struct.unpack(_NAV_CMD_FMT, raw)
                self._last_cmd_time = time.monotonic()
                twist = self._to_twist(mode, vel_scale, heading_offset, bool(safety_override))
                self.cmd_vel_pub.publish(twist)
            except zmq.Again:
                pass   # poll timeout — watchdog handles silence
            except Exception as exc:
                self.get_logger().error(f'[nav_sub] recv error: {exc}')

    # ── Watchdog ─────────────────────────────────────────────────────────────
    def _watchdog_tick(self) -> None:
        if time.monotonic() - self._last_cmd_time > WATCHDOG_TIMEOUT_SEC:
            self.cmd_vel_pub.publish(Twist())

    # ── NavigationMode + velocity_scale → Twist ──────────────────────────────
    @staticmethod
    def _to_twist(mode: int, vel_scale: float, heading_offset: float,
                  safety_override: bool) -> Twist:
        twist = Twist()
        if mode == NAV_STOP:
            return twist

        if mode == NAV_AVOID:
            scale = min(vel_scale, 0.3)
        elif mode == NAV_CAUTIOUS:
            scale = min(vel_scale, 0.6)
        else:
            scale = vel_scale   # CRUISE / FOLLOW

        twist.linear.x  = scale * MAX_LINEAR_VEL
        twist.angular.z = math.tan(heading_offset) * MAX_ANGULAR_VEL
        return twist

    # ── RobotState encoding (Protobuf preferred, struct fallback) ────────────
    def _encode_robot_state(self, odom: dict) -> bytes:
        if self._pb:
            try:
                msg = self._pb.RobotState()
                msg.vx     = odom.get('vx', 0.0)
                msg.vy     = odom.get('vy', 0.0)
                msg.vtheta = odom.get('vtheta', 0.0)
                msg.pos_x  = odom.get('pos_x', 0.0)
                msg.pos_y  = odom.get('pos_y', 0.0)
                msg.pos_theta = odom.get('pos_theta', 0.0)
                msg.battery_percent = self._battery_percent
                msg.nav2_status = 'idle'
                msg.timestamp = time.time()
                return msg.SerializeToString()
            except Exception as exc:
                self.get_logger().debug(f'Proto encode failed, using struct: {exc}')

        # struct fallback — Jetson zmq_subscriber._decode() accepts this at 36 bytes
        return struct.pack(
            _ROBOT_STATE_FMT,
            odom.get('vx', 0.0),
            odom.get('vy', 0.0),
            odom.get('vtheta', 0.0),
            odom.get('pos_x', 0.0),
            odom.get('pos_y', 0.0),
            odom.get('pos_theta', 0.0),
            self._battery_percent,          # battery_percent (real value from /PowerVoltage)
            time.time(),  # timestamp (double)
        )

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self) -> None:
        self._nav_sub.close()
        self._state_pub.close()
        self._ctx.term()
        super().destroy_node()
