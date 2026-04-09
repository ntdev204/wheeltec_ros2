#!/usr/bin/env python3
"""YOLOv8m TensorRT detection node for object detection."""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import time
from pathlib import Path
import yaml

from wheeltec_robot_msg.msg import Detection2D, Detection2DArray
from wheeltec_robot_detection.inference.tensorrt_detector import TensorRTDetector


class DetectionNode(Node):
    """ROS2 node for YOLOv8m object detection using TensorRT."""

    def __init__(self):
        super().__init__('detection_node')

        # Declare parameters
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        # Load configuration
        if config_file and Path(config_file).exists():
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.get_logger().warn('No config file provided, using defaults')
            self.config = self._default_config()

        detection_config = self.config['detection']
        topics_config = self.config['topics']

        # Initialize detector
        self.get_logger().info(f"Loading model from {detection_config['model_path']}")
        try:
            self.detector = TensorRTDetector(
                model_path=detection_config['model_path'],
                input_size=tuple(detection_config['input_size']),
                confidence_threshold=detection_config['confidence_threshold'],
                nms_threshold=detection_config['nms_threshold'],
                class_names=detection_config['class_names']
            )
            self.get_logger().info('TensorRT detector loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load detector: {e}')
            raise

        # CV Bridge
        self.bridge = CvBridge()

        # QoS profile for camera (best effort)
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            topics_config['camera_input'],
            self.image_callback,
            camera_qos
        )

        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            topics_config['detections_output'],
            10
        )

        self.fps_pub = self.create_publisher(
            Float32,
            topics_config['fps_output'],
            10
        )

        self.latency_pub = self.create_publisher(
            Float32,
            topics_config['latency_output'],
            10
        )

        # Pre-allocate reusable preprocessing buffer (avoids per-frame malloc)
        input_w, input_h = detection_config['input_size']
        self._preprocess_buf = np.zeros((1, 3, input_h, input_w), dtype=np.float32)

        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        self.get_logger().info('Detection node initialized')

    def _default_config(self):
        """Return default configuration."""
        return {
            'detection': {
                'model_path': '/home/robot/wheeltec_ros2/src/wheeltec_robot_detection/models/yolov8m.engine',
                'input_size': [640, 640],
                'confidence_threshold': 0.5,
                'nms_threshold': 0.45,
                'class_names': ['person', 'bicycle', 'car']  # Truncated for default
            },
            'topics': {
                'camera_input': '/camera/color/image_raw',
                'detections_output': '/detections',
                'fps_output': '/ai/fps',
                'latency_output': '/ai/latency'
            }
        }

    def image_callback(self, msg: Image):
        """Process incoming image and run detection."""
        start_time = time.time()

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run detection
            detections = self.detector.detect(cv_image)

            # Create detection message
            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for det in detections:
                detection_msg = Detection2D()
                detection_msg.header = msg.header
                detection_msg.class_name = det['class_name']
                detection_msg.class_id = det['class_id']
                detection_msg.confidence = det['confidence']

                # Pixel coordinates
                x_min, y_min, x_max, y_max = det['bbox']
                detection_msg.x_min = x_min
                detection_msg.y_min = y_min
                detection_msg.x_max = x_max
                detection_msg.y_max = y_max

                # Normalized coordinates
                img_h, img_w = cv_image.shape[:2]
                detection_msg.x_center = (x_min + x_max) / 2.0 / img_w
                detection_msg.y_center = (y_min + y_max) / 2.0 / img_h
                detection_msg.width = (x_max - x_min) / img_w
                detection_msg.height = (y_max - y_min) / img_h

                detection_array.detections.append(detection_msg)

            # Publish detections
            self.detections_pub.publish(detection_array)

            # Calculate and publish latency
            latency = (time.time() - start_time) * 1000  # ms
            latency_msg = Float32()
            latency_msg.data = latency
            self.latency_pub.publish(latency_msg)

            # Update FPS
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (current_time - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = current_time

                fps_msg = Float32()
                fps_msg.data = self.fps
                self.fps_pub.publish(fps_msg)

                self.get_logger().info(
                    f'FPS: {self.fps:.1f}, Latency: {latency:.1f}ms, Detections: {len(detections)}'
                )

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    # SingleThreadedExecutor: keeps CUDA context on the same thread that
    # created it in TensorRTDetector.__init__ (make_context pushes to caller
    # thread). GPU work is already async via CUDA stream — no CPU threading needed.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
