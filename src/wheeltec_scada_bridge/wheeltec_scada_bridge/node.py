import math
from datetime import datetime, timezone
from threading import RLock, Thread

import cv2
import numpy as np
import rclpy
import tf2_ros
import zmq
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Float32

MAX_LINEAR_VELOCITY = 1.0
MAX_ANGULAR_VELOCITY = 2.0
MAX_PATROL_WAYPOINT_TOLERANCE = 2.0
MIN_PATROL_WAYPOINT_TOLERANCE = 0.05


class WheeltecControlNode(Node):
    def __init__(self, zmq_ports, camera_topic='/camera/color/image_raw'):
        super().__init__('scada_control_node')

        # Declare ROS2 parameters for runtime configurability
        self.declare_parameter('zmq_cmd_port', zmq_ports.get('cmd', 5555))
        self.declare_parameter('zmq_telemetry_port', zmq_ports.get('telemetry', 5556))
        self.declare_parameter('zmq_camera_port', zmq_ports.get('camera', 5557))
        self.declare_parameter('camera_topic', camera_topic)

        cmd_port = self.get_parameter('zmq_cmd_port').value
        telemetry_port = self.get_parameter('zmq_telemetry_port').value
        camera_port = self.get_parameter('zmq_camera_port').value
        cam_topic = self.get_parameter('camera_topic').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(Imu, 'imu/data_raw', self.imu_cb, 10)
        self.create_subscription(Float32, '/PowerVoltage', self.voltage_cb, 10)
        self.create_subscription(Bool, '/robot_charging_flag', self.charging_cb, 10)

        # TF2 for robot localization in map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /map topic uses TRANSIENT_LOCAL + RELIABLE (latched in Nav2)
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        # Subscribe to Nav2 planned path and local path
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, 10)

        cam_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Image, cam_topic, self.camera_cb, cam_qos)

        # ZMQ Context setup
        self.zmq_context = zmq.Context()

        # Telemetry PUB socket (bind)
        self.telemetry_pub = self.zmq_context.socket(zmq.PUB)
        self.telemetry_pub.bind(f"tcp://0.0.0.0:{telemetry_port}")

        # Camera PUB socket (bind)
        self.camera_pub = self.zmq_context.socket(zmq.PUB)
        self.camera_pub.bind(f"tcp://0.0.0.0:{camera_port}")

        # Command REP socket (run in separate thread)
        self.cmd_rep = self.zmq_context.socket(zmq.REP)
        self.cmd_rep.bind(f"tcp://0.0.0.0:{cmd_port}")

        self.telemetry_data = {
            "odom": {"x": 0.0, "y": 0.0, "z": 0.0, "v_x": 0.0, "v_y": 0.0, "v_z": 0.0, "yaw": 0.0},
            "map_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "imu": {"ax": 0.0, "ay": 0.0, "az": 0.0},
            "voltage": 0.0,
            "charging": False,
            "plan": [],
            "local_plan": [],
            "patrol": {
                "status": "idle",
                "run_id": None,
                "schedule_id": None,
                "route_id": None,
                "current_loop": 0,
                "total_loops": 0,
                "current_waypoint_index": -1,
                "total_waypoints": 0,
                "last_goal": None,
                "message": None,
                "updated_at": None,
            }
        }

        self.map_data = None
        self.map_dirty = False
        self._last_map_msg = None

        # Patrol runtime state
        self._patrol_status = "idle"
        self._patrol_phase = "idle"  # idle|go_home_start|patrol|return_home_end
        self._patrol_run_id = None
        self._patrol_schedule_id = None
        self._patrol_route_id = None
        self._patrol_waypoints = []
        self._patrol_home = None
        self._patrol_total_loops = 0
        self._patrol_current_loop = 0
        self._patrol_current_waypoint_index = -1
        self._patrol_waypoint_tolerance = 0.25
        self._patrol_goal_sent_at = None
        self._patrol_waypoint_timeout_sec = 180.0
        self._patrol_last_goal = None
        self._patrol_message = None
        self._patrol_lock = RLock()

        # Start command listener thread
        self.cmd_thread = Thread(target=self.cmd_loop, daemon=True)
        self.cmd_thread.start()

        # Timers
        self.create_timer(0.1, self.publish_telemetry)
        self.create_timer(2.0, self.publish_map)
        self.create_timer(0.1, self.update_map_pose)
        self.create_timer(0.2, self.patrol_tick)

    def odom_cb(self, msg):
        self.telemetry_data["odom"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "v_x": msg.twist.twist.linear.x,
            "v_y": msg.twist.twist.linear.y,
            "v_z": msg.twist.twist.angular.z,
            "yaw": self._yaw_from_quat(msg.pose.pose.orientation)
        }

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quat_from_yaw(yaw: float) -> tuple[float, float]:
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _now_iso(self) -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _safe_float(value, field_name: str) -> float:
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ValueError(f"{field_name} must be finite")
        return numeric

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def _set_patrol_state(self, status: str, message: str | None = None):
        with self._patrol_lock:
            self._patrol_status = status
            self._patrol_message = message
            self.telemetry_data["patrol"] = {
                "status": self._patrol_status,
                "run_id": self._patrol_run_id,
                "schedule_id": self._patrol_schedule_id,
                "route_id": self._patrol_route_id,
                "current_loop": self._patrol_current_loop,
                "total_loops": self._patrol_total_loops,
                "current_waypoint_index": self._patrol_current_waypoint_index,
                "total_waypoints": len(self._patrol_waypoints),
                "last_goal": self._patrol_last_goal,
                "message": message,
                "updated_at": self._now_iso(),
            }

    def _is_patrol_active(self) -> bool:
        with self._patrol_lock:
            return self._patrol_status in {"starting", "running", "returning_home"}

    def _publish_goal(self, x: float, y: float, yaw: float = 0.0):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        qz, qw = self._quat_from_yaw(float(yaw))
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.goal_pub.publish(goal)
        with self._patrol_lock:
            self._patrol_last_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._patrol_goal_sent_at = self.get_clock().now().nanoseconds / 1e9

    def _start_patrol(self, payload: dict):
        waypoints = payload.get("waypoints") or []
        loops = int(payload.get("loops") or 0)
        home = payload.get("home")
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            raise ValueError("Patrol requires at least 2 waypoints")
        if loops <= 0:
            raise ValueError("Patrol loops must be > 0")
        if not isinstance(home, dict):
            raise ValueError("Patrol home is required")

        normalized_waypoints = []
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError(f"Waypoint {index} must be an object")
            normalized_waypoints.append({
                "x": self._safe_float(waypoint.get("x"), f"waypoints[{index}].x"),
                "y": self._safe_float(waypoint.get("y"), f"waypoints[{index}].y"),
                "yaw": self._safe_float(waypoint.get("yaw", 0.0), f"waypoints[{index}].yaw"),
            })

        normalized_home = {
            "x": self._safe_float(home.get("x"), "home.x"),
            "y": self._safe_float(home.get("y"), "home.y"),
            "yaw": self._safe_float(home.get("yaw", 0.0), "home.yaw"),
        }
        waypoint_tolerance = self._clamp(
            self._safe_float(payload.get("waypoint_tolerance", 0.25), "waypoint_tolerance"),
            MIN_PATROL_WAYPOINT_TOLERANCE,
            MAX_PATROL_WAYPOINT_TOLERANCE,
        )

        with self._patrol_lock:
            if self._patrol_status in {"starting", "running", "returning_home"}:
                raise ValueError("Patrol mission already running")
            self._patrol_run_id = payload.get("run_id")
            self._patrol_schedule_id = payload.get("schedule_id")
            self._patrol_route_id = payload.get("route_id")
            self._patrol_waypoints = normalized_waypoints
            self._patrol_home = normalized_home
            self._patrol_total_loops = loops
            self._patrol_current_loop = 0
            self._patrol_current_waypoint_index = -1
            self._patrol_waypoint_tolerance = waypoint_tolerance
            self._patrol_phase = "go_home_start"

        self._set_patrol_state("starting", "Navigating to home before patrol")
        self._publish_goal(normalized_home["x"], normalized_home["y"], normalized_home["yaw"])
        self.get_logger().info(
            f"Started patrol run={self._patrol_run_id} loops={self._patrol_total_loops} waypoints={len(self._patrol_waypoints)}"
        )

    def _stop_patrol(self, status: str, message: str):
        with self._patrol_lock:
            self._patrol_phase = "idle"
            self._patrol_waypoints = []
            self._patrol_total_loops = 0
            self._patrol_current_loop = 0
            self._patrol_current_waypoint_index = -1
            self._patrol_goal_sent_at = None
        self._set_patrol_state(status, message)
        self.get_logger().info(f"Patrol status={status}: {message}")

    def _distance_to_current_goal(self) -> float | None:
        with self._patrol_lock:
            goal = dict(self._patrol_last_goal) if self._patrol_last_goal else None
        pose = self.telemetry_data.get("map_pose")
        if not goal or not pose:
            return None
        dx = float(pose.get("x", 0.0)) - float(goal.get("x", 0.0))
        dy = float(pose.get("y", 0.0)) - float(goal.get("y", 0.0))
        return math.sqrt(dx * dx + dy * dy)

    def _advance_patrol_goal(self):
        with self._patrol_lock:
            phase = self._patrol_phase
            total_loops = self._patrol_total_loops
            waypoints = [dict(item) for item in self._patrol_waypoints]
            current_loop = self._patrol_current_loop
            current_waypoint_index = self._patrol_current_waypoint_index
            home = dict(self._patrol_home) if self._patrol_home else None

        if phase == "go_home_start":
            if not waypoints:
                self._stop_patrol("failed", "No patrol waypoints configured")
                return
            with self._patrol_lock:
                self._patrol_phase = "patrol"
                self._patrol_current_loop = 1
                self._patrol_current_waypoint_index = 0
            waypoint = waypoints[0]
            self._set_patrol_state("running", f"Loop 1/{total_loops}, waypoint 1/{len(waypoints)}")
            self._publish_goal(waypoint["x"], waypoint["y"], waypoint.get("yaw", 0.0))
            return

        if phase == "patrol":
            if current_waypoint_index < len(waypoints) - 1:
                next_index = current_waypoint_index + 1
                with self._patrol_lock:
                    self._patrol_current_waypoint_index = next_index
                waypoint = waypoints[next_index]
                self._set_patrol_state(
                    "running",
                    f"Loop {current_loop}/{total_loops}, waypoint {next_index + 1}/{len(waypoints)}"
                )
                self._publish_goal(waypoint["x"], waypoint["y"], waypoint.get("yaw", 0.0))
                return

            if current_loop < total_loops:
                next_loop = current_loop + 1
                with self._patrol_lock:
                    self._patrol_current_loop = next_loop
                    self._patrol_current_waypoint_index = 0
                waypoint = waypoints[0]
                self._set_patrol_state(
                    "running",
                    f"Loop {next_loop}/{total_loops}, waypoint 1/{len(waypoints)}"
                )
                self._publish_goal(waypoint["x"], waypoint["y"], waypoint.get("yaw", 0.0))
                return

            if home is None:
                self._stop_patrol("failed", "Home position is missing during return phase")
                return
            with self._patrol_lock:
                self._patrol_phase = "return_home_end"
            self._set_patrol_state("returning_home", "All loops complete, returning home")
            self._publish_goal(home["x"], home["y"], home["yaw"])
            return

        if phase == "return_home_end":
            self._stop_patrol("completed", "Patrol completed and returned home")

    def patrol_tick(self):
        if not self._is_patrol_active():
            return

        with self._patrol_lock:
            waypoint_tolerance = self._patrol_waypoint_tolerance
            goal_sent_at = self._patrol_goal_sent_at

        dist = self._distance_to_current_goal()
        if dist is not None and dist <= waypoint_tolerance:
            self._advance_patrol_goal()
            return

        if goal_sent_at is not None:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - goal_sent_at > self._patrol_waypoint_timeout_sec:
                self._stop_patrol("failed", "Timeout while waiting to reach patrol goal")

    def imu_cb(self, msg):
        self.telemetry_data["imu"] = {
            "ax": msg.linear_acceleration.x,
            "ay": msg.linear_acceleration.y,
            "az": msg.linear_acceleration.z
        }

    def voltage_cb(self, msg):
        self.telemetry_data["voltage"] = msg.data

    def charging_cb(self, msg):
        self.telemetry_data["charging"] = msg.data

    def plan_cb(self, msg):
        poses = msg.poses
        step = max(1, len(poses) // 100)
        self.telemetry_data["plan"] = [
            {"x": p.pose.position.x, "y": p.pose.position.y}
            for p in poses[::step]
        ]

    def local_plan_cb(self, msg):
        self.telemetry_data["local_plan"] = [
            {"x": p.pose.position.x, "y": p.pose.position.y}
            for p in msg.poses
        ]

    def map_cb(self, msg):
        self._last_map_msg = msg
        self.get_logger().info(f'Received /map: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}')

        w = msg.info.width
        h = msg.info.height

        self.telemetry_data["map_info"] = {
            "resolution": msg.info.resolution,
            "width": w,
            "height": h,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y
            }
        }

        try:
            img_array = np.zeros((h, w), dtype=np.uint8)
            data = msg.data
            for i in range(len(data)):
                val = data[i]
                if val == -1:
                    img_array[i // w, i % w] = 205
                elif val == 0:
                    img_array[i // w, i % w] = 255
                else:
                    img_array[i // w, i % w] = max(0, 255 - int(val * 2.55))

            img_array = np.flipud(img_array)
            _, png_bytes = cv2.imencode('.png', img_array)
            self.camera_pub.send(b'MAP:' + png_bytes.tobytes())
            self.get_logger().info(f'Sent map PNG: {len(png_bytes)} bytes')
        except Exception as e:
            self.get_logger().error(f'Map PNG encode error: {e}')

        self.map_data = None
        self.map_dirty = False

    def update_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            yaw = self._yaw_from_quat(q)
            self.telemetry_data["map_pose"] = {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "yaw": yaw
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def camera_cb(self, msg):
        try:
            channels = 3 if '8' in msg.encoding else 1
            if msg.encoding in ['rgb8', 'bgr8', 'mono8']:
                image_np = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))

                if msg.encoding == 'rgb8':
                    image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
                result, encimg = cv2.imencode('.jpg', image_np, encode_param)

                if result:
                    self.camera_pub.send(encimg.tobytes())
        except Exception as e:
            self.get_logger().error(f"Image encode error: {e}")

    def publish_telemetry(self):
        self.telemetry_pub.send_json(self.telemetry_data)

    def publish_map(self):
        if self.map_data and self.map_dirty:
            self.telemetry_pub.send_json({"type": "map", "payload": self.map_data})
            self.map_dirty = False

    def cmd_loop(self):
        while rclpy.ok():
            try:
                msg = self.cmd_rep.recv_json()
                action = msg.get("action")
                payload = msg.get("payload", {})

                if action == "cmd_vel":
                    twist = Twist()
                    twist.linear.x = self._clamp(
                        self._safe_float(payload.get("linear_x", 0.0), "linear_x"),
                        -MAX_LINEAR_VELOCITY,
                        MAX_LINEAR_VELOCITY,
                    )
                    twist.linear.y = self._clamp(
                        self._safe_float(payload.get("linear_y", 0.0), "linear_y"),
                        -MAX_LINEAR_VELOCITY,
                        MAX_LINEAR_VELOCITY,
                    )
                    twist.angular.z = self._clamp(
                        self._safe_float(payload.get("angular_z", 0.0), "angular_z"),
                        -MAX_ANGULAR_VELOCITY,
                        MAX_ANGULAR_VELOCITY,
                    )
                    self.cmd_vel_pub.publish(twist)
                    self.cmd_rep.send_json({"status": "ok"})

                elif action == "nav_goal":
                    if self._is_patrol_active():
                        self._stop_patrol("aborted", "Interrupted by manual nav goal")
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x = self._safe_float(payload.get("x", 0.0), "x")
                    goal.pose.position.y = self._safe_float(payload.get("y", 0.0), "y")
                    yaw = self._safe_float(payload.get("theta", payload.get("yaw", 0.0)), "yaw")
                    qz, qw = self._quat_from_yaw(yaw)
                    goal.pose.orientation.z = qz
                    goal.pose.orientation.w = qw
                    self.goal_pub.publish(goal)
                    self.get_logger().info(f"Published nav goal to x={goal.pose.position.x}, y={goal.pose.position.y}, yaw={yaw}")
                    self.cmd_rep.send_json({"status": "goal_sent"})

                elif action == "patrol_start":
                    self._start_patrol(payload)
                    with self._patrol_lock:
                        run_id = self._patrol_run_id
                    self.cmd_rep.send_json({"status": "started", "run_id": run_id})

                elif action == "patrol_stop":
                    reason = str(payload.get("reason") or "Patrol stopped by command")
                    if self._is_patrol_active():
                        self._stop_patrol("stopped", reason)
                    with self._patrol_lock:
                        run_id = self._patrol_run_id
                    self.cmd_rep.send_json({"status": "stopped", "run_id": run_id})

                elif action == "resend_map":
                    if self._last_map_msg is not None:
                        self.map_cb(self._last_map_msg)
                        self.cmd_rep.send_json({"status": "map_resent"})
                    else:
                        self.cmd_rep.send_json({"status": "no_map_available"})

                elif action == "slam_control":
                    self.cmd_rep.send_json({"status": "pending_implementation"})
                else:
                    self.cmd_rep.send_json({"status": "unknown_action"})
            except Exception as e:
                self.get_logger().error(f"ZMQ cmd error: {e}")
                try:
                    self.cmd_rep.send_json({"status": "error", "message": str(e)})
                except Exception:
                    pass
