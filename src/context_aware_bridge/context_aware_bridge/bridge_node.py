
import json
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

from context_aware_bridge.obstacle_guard import ObstacleGuard

NAV_CRUISE   = 0
NAV_CAUTIOUS = 1
NAV_AVOID    = 2
NAV_RESERVED = 3
NAV_STOP     = 4

_NAV_CMD_FMT  = '!ifffiffB'
_NAV_CMD_SIZE = struct.calcsize(_NAV_CMD_FMT)

MAX_LINEAR_VEL  = 0.8
MAX_ANGULAR_VEL = 1.5

WATCHDOG_TIMEOUT_SEC = 1.5
LIDAR_MAX_DISTANCE = 9.9
LIDAR_SCAN_BINS = 360


class ContextAwareBridgeNode(Node):
    def __init__(self):
        super().__init__('context_aware_bridge')

        self.declare_parameter('jetson_ip',        '25.12.4.100')
        self.declare_parameter('raspi_ip',         '25.12.4.101')
        self.declare_parameter('nav_cmd_port',     5555)
        self.declare_parameter('robot_state_port', 5560)
        self.declare_parameter('lidar_stop_distance', 0.30)
        self.declare_parameter('lidar_slow_distance', 0.60)
        self.declare_parameter('scan_stale_timeout', 0.75)

        jetson_ip        = self.get_parameter('jetson_ip').value
        robot_state_port = self.get_parameter('robot_state_port').value
        nav_cmd_port     = self.get_parameter('nav_cmd_port').value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_context', 10)

        odom_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._odom_cb, odom_qos)
        self.create_subscription(Float32, '/PowerVoltage', self._voltage_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, rclpy.qos.qos_profile_sensor_data)

        self._ctx = zmq.Context()

        self._nav_sub = self._ctx.socket(zmq.SUB)
        self._nav_sub.setsockopt(zmq.RCVHWM, 2)
        self._nav_sub.setsockopt(zmq.LINGER, 0)
        self._nav_sub.setsockopt(zmq.RCVTIMEO, 200)
        self._nav_sub.setsockopt_string(zmq.SUBSCRIBE, 'ai/nav_cmd')
        self._nav_sub.connect(f'tcp://{jetson_ip}:{nav_cmd_port}')

        self._state_pub = self._ctx.socket(zmq.PUB)
        self._state_pub.setsockopt(zmq.SNDHWM, 5)
        self._state_pub.setsockopt(zmq.LINGER, 0)
        self._state_pub.bind(f'tcp://0.0.0.0:{robot_state_port}')

        self._last_cmd_time = time.monotonic()
        self._yielding_to_nav2 = False
        self._watchdog_stop_sent = False
        self._odom: dict = {}
        self._battery_percent = 0.0
        self._lidar_sectors = [9.9, 9.9, 9.9, 9.9]
        self._lidar_scan360 = [LIDAR_MAX_DISTANCE] * LIDAR_SCAN_BINS
        self._lidar_valid = False
        self._last_scan_time = 0.0
        self._obstacle_guard = ObstacleGuard(
            stop_distance=self.get_parameter('lidar_stop_distance').value,
            slow_distance=self.get_parameter('lidar_slow_distance').value,
            scan_stale_timeout=self.get_parameter('scan_stale_timeout').value,
        )

        self._recv_thread = Thread(target=self._recv_loop, daemon=True, name='zmq-nav-sub')
        self._recv_thread.start()

        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            f'[context_aware_bridge] started\n'
            f'  SUB  tcp://{jetson_ip}:{nav_cmd_port}  (ai/nav_cmd)\n'
            f'  PUB  tcp://0.0.0.0:{robot_state_port}  (robot/state)'
        )

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

    def _voltage_cb(self, msg: Float32) -> None:
        voltage = msg.data
        if voltage > 25.2:
            self._battery_percent = 100.0
        elif voltage < 21.0:
            self._battery_percent = 0.0
        else:
            self._battery_percent = (voltage - 21.0) / 4.2 * 100.0

    def _scan_cb(self, msg: LaserScan) -> None:
        f, r, l, ri = [], [], [], []
        scan360 = [LIDAR_MAX_DISTANCE] * LIDAR_SCAN_BINS
        angle = msg.angle_min
        for dist in msg.ranges:
            if msg.range_min < dist < msg.range_max:
                deg = math.degrees(angle)
                idx = int(round(deg)) % LIDAR_SCAN_BINS
                scan360[idx] = min(scan360[idx], min(float(dist), LIDAR_MAX_DISTANCE))
                if -45 <= deg <= 45: f.append(dist)
                elif deg >= 135 or deg <= -135: r.append(dist)
                elif 45 < deg < 135: l.append(dist)
                elif -135 < deg < -45: ri.append(dist)
            angle += msg.angle_increment
        
        self._lidar_valid = bool(f or r or l or ri)
        self._lidar_sectors = [
            min(f) if f else 9.9,
            min(r) if r else 9.9,
            min(l) if l else 9.9,
            min(ri) if ri else 9.9
        ]
        self._lidar_scan360 = scan360
        self._last_scan_time = time.monotonic()

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
                self._watchdog_stop_sent = False

                if mode == NAV_STOP and not bool(safety_override):
                    self._yielding_to_nav2 = True
                    continue

                self._yielding_to_nav2 = False
                twist = self._to_twist(mode, vel_x, vel_y, heading_offset, bool(safety_override))
                scan_age_s = time.monotonic() - self._last_scan_time
                lidar_sectors = self._lidar_sectors if self._lidar_valid else None
                twist = self._obstacle_guard.guard(twist, lidar_sectors, scan_age_s)
                self.cmd_vel_pub.publish(twist)
            except zmq.Again:
                pass
            except Exception as exc:
                self.get_logger().error(f'[nav_sub] recv error: {exc}')

    def _watchdog_tick(self) -> None:
        if self._yielding_to_nav2:
            return
        if (
            time.monotonic() - self._last_cmd_time > WATCHDOG_TIMEOUT_SEC
            and not self._watchdog_stop_sent
        ):
            self.cmd_vel_pub.publish(Twist())
            self._watchdog_stop_sent = True

    @staticmethod
    def _to_twist(mode: int, vel_x: float, vel_y: float, heading_offset: float,
                  safety_override: bool) -> Twist:
        twist = Twist()
        if mode in (NAV_STOP, NAV_RESERVED):
            return twist

        if mode == NAV_AVOID:
            scale_x = min(vel_x, 0.3)
        elif mode == NAV_CAUTIOUS:
            scale_x = min(vel_x, 0.6)
        else:
            scale_x = vel_x

        twist.linear.x  = scale_x
        twist.linear.y  = vel_y
        twist.angular.z = heading_offset * MAX_ANGULAR_VEL
        return twist

    def _encode_robot_state(self, odom: dict) -> bytes:
        return json.dumps(
            {
                'odom': {
                    'vx': odom.get('vx', 0.0),
                    'vy': odom.get('vy', 0.0),
                    'vtheta': odom.get('vtheta', 0.0),
                    'pos_x': odom.get('pos_x', 0.0),
                    'pos_y': odom.get('pos_y', 0.0),
                    'pos_theta': odom.get('pos_theta', 0.0),
                },
                'battery_percent': self._battery_percent,
                'nav2_status': 'idle',
                'lidar': {
                    'sectors': {
                        'front': self._lidar_sectors[0],
                        'rear': self._lidar_sectors[1],
                        'left': self._lidar_sectors[2],
                        'right': self._lidar_sectors[3],
                    },
                    # 360 bins in robot frame: 0=front, 90=left, 180=rear, 270=right.
                    'scan360': [round(v, 3) for v in self._lidar_scan360],
                },
                'timestamp': time.time(),
            },
            separators=(',', ':'),
        ).encode('utf-8')

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
