import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32, Bool
import cv2
import numpy as np
import zmq
from threading import Thread

class WheeltecControlNode(Node):
    def __init__(self, zmq_ports):
        super().__init__('scada_control_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(Imu, 'imu/data_raw', self.imu_cb, 10)
        self.create_subscription(Float32, 'PowerVoltage', self.voltage_cb, 10)
        self.create_subscription(Bool, 'robot_charging_flag', self.charging_cb, 10)
        
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscribe to camera topic
        self.camera_topic = "/camera/color/image_raw"
        self.create_subscription(Image, self.camera_topic, self.camera_cb, qos_profile)
        
        # ZMQ Context setup
        self.zmq_context = zmq.Context()
        
        # Telemetry PUB socket (bind)
        self.telemetry_pub = self.zmq_context.socket(zmq.PUB)
        self.telemetry_pub.bind(f"tcp://0.0.0.0:{zmq_ports['telemetry']}")
        
        # Camera PUB socket (bind)
        self.camera_pub = self.zmq_context.socket(zmq.PUB)
        self.camera_pub.bind(f"tcp://0.0.0.0:{zmq_ports['camera']}")
        
        # Command REP socket (run in separate thread since rclpy spins in main thread)
        self.cmd_rep = self.zmq_context.socket(zmq.REP)
        self.cmd_rep.bind(f"tcp://0.0.0.0:{zmq_ports['cmd']}")
        
        self.telemetry_data = {
            "odom": {"x": 0.0, "y": 0.0, "z": 0.0, "v_x": 0.0, "v_y": 0.0, "v_z": 0.0},
            "imu": {"ax": 0.0, "ay": 0.0, "az": 0.0},
            "voltage": 0.0,
            "charging": False
        }
        
        # Start command listener thread
        self.cmd_thread = Thread(target=self.cmd_loop, daemon=True)
        self.cmd_thread.start()
        
        # Timer to send telemetry at 10Hz
        self.create_timer(0.1, self.publish_telemetry)

    def odom_cb(self, msg):
        self.telemetry_data["odom"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "v_x": msg.twist.twist.linear.x,
            "v_y": msg.twist.twist.linear.y,
            "v_z": msg.twist.twist.angular.z
        }

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

    def camera_cb(self, msg):
        # Compress raw image to JPEG on ROS2 side, optimized for RPi 4
        try:
            channels = 3 if '8' in msg.encoding else 1
            if msg.encoding in ['rgb8', 'bgr8', 'mono8']:
                image_np = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
                
                if msg.encoding == 'rgb8':
                    image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
                
                # Compress to JPEG with moderate quality
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
                result, encimg = cv2.imencode('.jpg', image_np, encode_param)
                
                if result:
                    # Send bytes via ZMQ PUB
                    self.camera_pub.send(encimg.tobytes())
        except Exception as e:
            self.get_logger().error(f"Image encode error: {e}")

    def publish_telemetry(self):
        self.telemetry_pub.send_json(self.telemetry_data)

    def cmd_loop(self):
        while rclpy.ok():
            try:
                # Wait for command from FastAPI
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
                    
                elif action == "slam_control":
                    # Pending lifecycle manager implementation
                    self.cmd_rep.send_json({"status": "pending_implementation"})
                else:
                    self.cmd_rep.send_json({"status": "unknown_action"})
            except Exception as e:
                self.get_logger().error(f"ZMQ cmd error: {e}")
