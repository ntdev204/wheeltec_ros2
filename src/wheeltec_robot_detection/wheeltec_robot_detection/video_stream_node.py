#!/usr/bin/env python3
"""Video streaming node with dual ZMQ streams for AI detection and human tracking."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
import yaml
import cv2
import zmq
import numpy as np

from wheeltec_robot_msg.msg import Detection2DArray, TrackedHumanArray


class VideoStreamNode(Node):
    """ROS2 node for streaming annotated video via ZMQ."""

    def __init__(self):
        super().__init__('video_stream_node')

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

        streaming_config = self.config['streaming']
        topics_config = streaming_config['topics']

        # ZMQ setup
        self.zmq_context = zmq.Context()

        # AI Detection stream (all objects)
        self.detection_pub = self.zmq_context.socket(zmq.PUB)
        self.detection_pub.bind(f"tcp://0.0.0.0:{streaming_config['ai_detection_port']}")

        # Human Tracking stream (humans only)
        self.tracking_pub = self.zmq_context.socket(zmq.PUB)
        self.tracking_pub.bind(f"tcp://0.0.0.0:{streaming_config['human_tracking_port']}")

        # Encoding settings
        self.format = streaming_config['format']
        self.quality = streaming_config['quality']
        self.encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]

        # Annotation settings
        self.bbox_thickness = streaming_config['bbox_thickness']
        self.font_scale = streaming_config['font_scale']
        self.font_thickness = streaming_config['font_thickness']
        self.colors = streaming_config['colors']

        # CV Bridge
        self.bridge = CvBridge()

        # Cache
        self.latest_image = None
        self.latest_detections = None
        self.latest_humans = None

        # QoS profiles
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

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            topics_config['detections_input'],
            self.detection_callback,
            10
        )

        self.human_sub = self.create_subscription(
            TrackedHumanArray,
            topics_config['tracked_humans_input'],
            self.human_callback,
            10
        )

        # Timer for streaming
        target_fps = streaming_config['target_fps']
        self.create_timer(1.0 / target_fps, self.stream_callback)

        self.get_logger().info(
            f'Video streaming initialized (ports: {streaming_config["ai_detection_port"]}, '
            f'{streaming_config["human_tracking_port"]})'
        )

    def _default_config(self):
        """Return default configuration."""
        return {
            'streaming': {
                'ai_detection_port': 5558,
                'human_tracking_port': 5559,
                'format': 'jpg',
                'quality': 75,
                'target_fps': 30,
                'bbox_thickness': 2,
                'font_scale': 0.5,
                'font_thickness': 1,
                'colors': {
                    'person': [0, 255, 0],
                    'vehicle': [255, 0, 0],
                    'default': [0, 255, 255]
                },
                'topics': {
                    'camera_input': '/camera/color/image_raw',
                    'detections_input': '/detections',
                    'tracked_humans_input': '/tracked_humans'
                }
            }
        }

    def image_callback(self, msg: Image):
        """Cache latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def detection_callback(self, msg: Detection2DArray):
        """Cache latest detections."""
        self.latest_detections = msg

    def human_callback(self, msg: TrackedHumanArray):
        """Cache latest tracked humans."""
        self.latest_humans = msg

    def stream_callback(self):
        """Stream annotated frames via ZMQ."""
        if self.latest_image is None:
            return

        try:
            # Stream 1: AI Detection (all objects)
            if self.latest_detections is not None:
                detection_frame = self._annotate_detections(self.latest_image.copy())
                _, encoded = cv2.imencode(f'.{self.format}', detection_frame, self.encode_params)
                self.detection_pub.send(b'DETECTION:' + encoded.tobytes())

            # Stream 2: Human Tracking (humans only)
            if self.latest_humans is not None:
                tracking_frame = self._annotate_humans(self.latest_image.copy())
                _, encoded = cv2.imencode(f'.{self.format}', tracking_frame, self.encode_params)
                self.tracking_pub.send(b'TRACKING:' + encoded.tobytes())

        except Exception as e:
            self.get_logger().error(f'Streaming error: {e}')

    def _annotate_detections(self, image: np.ndarray) -> np.ndarray:
        """Annotate image with all detections."""
        for det in self.latest_detections.detections:
            x_min, y_min = det.x_min, det.y_min
            x_max, y_max = det.x_max, det.y_max

            # Get color
            if det.class_name == 'person':
                color = tuple(self.colors['person'])
            elif det.class_name in ['car', 'truck', 'bus', 'motorcycle', 'bicycle']:
                color = tuple(self.colors['vehicle'])
            else:
                color = tuple(self.colors['default'])

            # Draw bbox
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, self.bbox_thickness)

            # Draw label
            label = f'{det.class_name} {det.confidence:.2f}'
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, self.font_thickness
            )
            cv2.rectangle(image, (x_min, y_min - label_h - 5),
                        (x_min + label_w, y_min), color, -1)
            cv2.putText(image, label, (x_min, y_min - 5),
                      cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0, 0, 0), self.font_thickness)

        return image

    def _annotate_humans(self, image: np.ndarray) -> np.ndarray:
        """Annotate image with tracked humans only."""
        color = tuple(self.colors['person'])

        for human in self.latest_humans.humans:
            x_min, y_min = human.x_min, human.y_min
            x_max, y_max = human.x_max, human.y_max

            # Draw bbox
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, self.bbox_thickness)

            # Draw label with track ID
            label = f'ID:{human.track_id} {human.confidence:.2f}'
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, self.font_thickness
            )
            cv2.rectangle(image, (x_min, y_min - label_h - 5),
                        (x_min + label_w, y_min), color, -1)
            cv2.putText(image, label, (x_min, y_min - 5),
                      cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0, 0, 0), self.font_thickness)

            # Draw velocity arrow if moving
            if abs(human.velocity_x) > 1 or abs(human.velocity_y) > 1:
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                arrow_end_x = int(center_x + human.velocity_x * 5)
                arrow_end_y = int(center_y + human.velocity_y * 5)
                cv2.arrowedLine(image, (center_x, center_y), (arrow_end_x, arrow_end_y),
                              color, 2, tipLength=0.3)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
