import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import cv2
import numpy as np
import zmq
from threading import Thread
import tf2_ros
import math


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
            "odom": {"x": 0.0, "y": 0.0, "z": 0.0, "v_x": 0.0, "v_y": 0.0, "v_z": 0.0},
            "map_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "imu": {"ax": 0.0, "ay": 0.0, "az": 0.0},
            "voltage": 0.0,
            "charging": False,
            "plan": [],
            "local_plan": []
        }

        self.map_data = None
        self.map_dirty = False
        self._last_map_msg = None

        # Start command listener thread
        self.cmd_thread = Thread(target=self.cmd_loop, daemon=True)
        self.cmd_thread.start()

        # Timers
        self.create_timer(0.1, self.publish_telemetry)
        self.create_timer(2.0, self.publish_map)
        self.create_timer(0.1, self.update_map_pose)

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
                    twist.linear.x = float(payload.get("linear_x", 0.0))
                    twist.linear.y = float(payload.get("linear_y", 0.0))
                    twist.angular.z = float(payload.get("angular_z", 0.0))
                    self.cmd_vel_pub.publish(twist)
                    self.cmd_rep.send_json({"status": "ok"})

                elif action == "nav_goal":
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x = float(payload.get("x", 0.0))
                    goal.pose.position.y = float(payload.get("y", 0.0))
                    goal.pose.orientation.z = float(payload.get("orien_z", 0.0))
                    goal.pose.orientation.w = float(payload.get("orien_w", 1.0))
                    self.goal_pub.publish(goal)
                    self.get_logger().info(f"Published nav goal to x={goal.pose.position.x}, y={goal.pose.position.y}, oz={goal.pose.orientation.z:.3f}, ow={goal.pose.orientation.w:.3f}")
                    self.cmd_rep.send_json({"status": "goal_sent"})

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
