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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# NavigationMode enum values (mirrors context-aware/proto/messages.proto)
NAV_CRUISE   = 0
NAV_CAUTIOUS = 1
NAV_AVOID    = 2
NAV_FOLLOW   = 3
NAV_STOP     = 4

# Struct format for NavigationCommand (Jetson → RasPi), 29 bytes big-endian
# Fields: mode(i) vx(f) vy(f) vtheta(f) follow_id(i) ts(f) confidence(f) safety_override(B)
_NAV_CMD_FMT  = '!ifffiffB'
_NAV_CMD_SIZE = struct.calcsize(_NAV_CMD_FMT)  # 29

# Struct format for RobotState fallback (RasPi → Jetson), 52 bytes big-endian
# Fields: vx(f) vy(f) vtheta(f) pos_x(f) pos_y(f) pos_theta(f) battery(f) 
#         dist_f(f) dist_r(f) dist_l(f) dist_ri(f) timestamp(d)
_ROBOT_STATE_FMT  = '!11fd'
_ROBOT_STATE_SIZE = struct.calcsize(_ROBOT_STATE_FMT)  # 52

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
        self.create_subscription(LaserScan, '/scan', self._scan_cb, rclpy.qos.qos_profile_sensor_data)

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
            self.get_logger().debug(
                'Protobuf stubs not found — using struct fallback for RobotState (expected).'
            )

        # ── Runtime state ────────────────────────────────────────────────────
        self._last_cmd_time = time.monotonic()
        self._odom: dict = {}
        self._battery_percent = 0.0  # Will be updated by /PowerVoltage callback
        self._lidar_sectors = [9.9, 9.9, 9.9, 9.9]  # F, R, L, Ri

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

    # ── /scan callback (Summarize into 4 sectors) ──────────────────────────
    def _scan_cb(self, msg: LaserScan) -> None:
        f, r, l, ri = [], [], [], []
        angle = msg.angle_min
        for dist in msg.ranges:
            if msg.range_min < dist < msg.range_max:
                deg = math.degrees(angle)
                if -45 <= deg <= 45: f.append(dist)
                elif deg >= 135 or deg <= -135: r.append(dist)
                elif 45 < deg < 135: l.append(dist)
                elif -135 < deg < -45: ri.append(dist)
            angle += msg.angle_increment
        
        self._lidar_sectors = [
            min(f) if f else 9.9,
            min(r) if r else 9.9,
            min(l) if l else 9.9,
            min(ri) if ri else 9.9
        ]

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
                mode, vel_x, vel_y, heading_offset, _, _, _, safety_override = \
                    struct.unpack(_NAV_CMD_FMT, raw)
                self._last_cmd_time = time.monotonic()
                twist = self._to_twist(mode, vel_x, vel_y, heading_offset, bool(safety_override))
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
    def _to_twist(mode: int, vel_x: float, vel_y: float, heading_offset: float,
                  safety_override: bool) -> Twist:
        twist = Twist()
        if mode == NAV_STOP:
            return twist

        if mode == NAV_AVOID:
            scale_x = min(vel_x, 0.3)
        elif mode == NAV_CAUTIOUS:
            scale_x = min(vel_x, 0.6)
        else:
            scale_x = vel_x   # CRUISE / FOLLOW

        # vel_x / vel_y are already in m/s from Jetson heuristic_policy
        # (range [follow_min_vel, follow_max_vel] = [0.3, 0.8] m/s)
        # Do NOT multiply by MAX_LINEAR_VEL — that would double-scale the velocity.
        twist.linear.x  = scale_x
        twist.linear.y  = vel_y
        twist.angular.z = heading_offset * MAX_ANGULAR_VEL
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
            self._battery_percent,
            self._lidar_sectors[0], self._lidar_sectors[1], 
            self._lidar_sectors[2], self._lidar_sectors[3],
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
