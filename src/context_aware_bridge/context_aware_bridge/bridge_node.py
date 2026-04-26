"""
context_aware_bridge — ROS2 node bridging the Jetson AI server to wheeltec_ros2.

Data flow:
  Jetson (ZMQ PUB :5555, topic b"ai/nav_cmd")
      → [NavigationCommand protobuf]
      → /cmd_vel_context  (geometry_msgs/Twist)   → twist_mux → /cmd_vel

  /odom (nav_msgs/Odometry)
      → [RobotState protobuf]
      → Jetson (ZMQ PUB :5560, topic b"robot/state")
"""

import math
import time
from threading import Thread

import rclpy
import zmq
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from context_aware_bridge.messages_pb2 import NavigationCommand, NavigationMode, RobotState

# ─── Velocity limits (metres/s and rad/s) ───────────────────────────────────
MAX_LINEAR_VEL = 0.8   # full-scale maps to this
MAX_ANGULAR_VEL = 1.5  # ±π/4 heading_offset maps to this

# ─── Safety: if no nav_cmd received within this many seconds, stop robot ──────
WATCHDOG_TIMEOUT_SEC = 1.5


class ContextAwareBridgeNode(Node):
    def __init__(self):
        super().__init__('context_aware_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('jetson_ip',       'raspberrypi.local')
        self.declare_parameter('nav_cmd_port',    5555)
        self.declare_parameter('robot_state_port',5560)

        jetson_ip        = self.get_parameter('jetson_ip').value
        nav_cmd_port     = self.get_parameter('nav_cmd_port').value
        robot_state_port = self.get_parameter('robot_state_port').value

        # ── ROS2 interfaces ─────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_context', 10)

        odom_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._odom_cb, odom_qos)

        # ── ZMQ setup ───────────────────────────────────────────────────────
        self._ctx = zmq.Context()

        # SUB: receive NavigationCommand from Jetson
        self._nav_sub = self._ctx.socket(zmq.SUB)
        self._nav_sub.setsockopt(zmq.RCVHWM, 2)
        self._nav_sub.setsockopt(zmq.LINGER, 0)
        self._nav_sub.setsockopt(zmq.RCVTIMEO, 200)          # ms, non-blocking poll
        self._nav_sub.setsockopt_string(zmq.SUBSCRIBE, 'ai/nav_cmd')
        self._nav_sub.connect(f'tcp://{jetson_ip}:{nav_cmd_port}')

        # PUB: publish RobotState to Jetson
        self._state_pub = self._ctx.socket(zmq.PUB)
        self._state_pub.setsockopt(zmq.SNDHWM, 5)
        self._state_pub.setsockopt(zmq.LINGER, 0)
        self._state_pub.bind(f'tcp://0.0.0.0:{robot_state_port}')

        # ── Runtime state ───────────────────────────────────────────────────
        self._last_cmd_time = time.monotonic()
        self._odom_state = RobotState()

        # ── Threads ─────────────────────────────────────────────────────────
        self._recv_thread = Thread(target=self._recv_loop, daemon=True, name='zmq-nav-sub')
        self._recv_thread.start()

        # Watchdog: publishes zero Twist if Jetson goes silent
        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            f'context_aware_bridge started | '
            f'SUB tcp://{jetson_ip}:{nav_cmd_port} | '
            f'PUB tcp://0.0.0.0:{robot_state_port}'
        )

    # ── Odometry callback → pack RobotState protobuf → ZMQ ─────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        s = self._odom_state
        s.vx     = msg.twist.twist.linear.x
        s.vy     = msg.twist.twist.linear.y
        s.vtheta = msg.twist.twist.angular.z
        s.pos_x  = msg.pose.pose.position.x
        s.pos_y  = msg.pose.pose.position.y
        s.pos_theta = self._yaw_from_quat(msg.pose.pose.orientation)
        s.timestamp = time.time()
        try:
            payload = s.SerializeToString()
            self._state_pub.send_multipart([b'robot/state', payload], flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    # ── ZMQ receive loop (background thread) ────────────────────────────────
    def _recv_loop(self) -> None:
        while rclpy.ok():
            try:
                parts = self._nav_sub.recv_multipart()
                if len(parts) < 2:
                    continue
                nav_cmd = NavigationCommand()
                nav_cmd.ParseFromString(parts[1])
                self._last_cmd_time = time.monotonic()
                twist = self._nav_cmd_to_twist(nav_cmd)
                self.cmd_vel_pub.publish(twist)
            except zmq.Again:
                pass   # timeout → loop again; watchdog handles stale stream
            except Exception as exc:
                self.get_logger().error(f'[context_aware_bridge] recv error: {exc}')

    # ── Watchdog: zero velocity if Jetson goes silent ───────────────────────
    def _watchdog_tick(self) -> None:
        if time.monotonic() - self._last_cmd_time > WATCHDOG_TIMEOUT_SEC:
            self.cmd_vel_pub.publish(Twist())   # zero = safe stop

    # ── NavigationCommand → geometry_msgs/Twist ─────────────────────────────
    @staticmethod
    def _nav_cmd_to_twist(cmd: NavigationCommand) -> Twist:
        twist = Twist()

        if cmd.mode == NavigationMode.STOP:
            return twist   # all zeros

        if cmd.mode == NavigationMode.AVOID:
            # When avoiding, reduce to 30 % max speed
            scale = min(cmd.velocity_scale, 0.3)
        elif cmd.mode == NavigationMode.CAUTIOUS:
            scale = min(cmd.velocity_scale, 0.6)
        else:
            scale = cmd.velocity_scale  # CRUISE / FOLLOW (0.0 – 1.0)

        twist.linear.x  = scale * MAX_LINEAR_VEL
        # heading_offset in radians [-π/4, π/4] → angular.z
        twist.angular.z = math.tan(cmd.heading_offset) * MAX_ANGULAR_VEL

        return twist

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
