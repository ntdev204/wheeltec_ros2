#!/usr/bin/env python3
"""Image storage node with 2k limit and FIFO management."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
import yaml
import cv2
import sqlite3
from datetime import datetime
import numpy as np

from wheeltec_robot_msg.msg import Detection2DArray


class ImageStorageNode(Node):
    """ROS2 node for storing images with object detections."""

    def __init__(self):
        super().__init__('image_storage_node')

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

        storage_config = self.config['storage']
        topics_config = storage_config['topics']

        # Setup storage directories
        self.base_path = Path(storage_config['base_path'])
        self.raw_path = self.base_path / storage_config['raw_subdir']
        self.annotated_path = self.base_path / storage_config['annotated_subdir']

        self.raw_path.mkdir(parents=True, exist_ok=True)
        self.annotated_path.mkdir(parents=True, exist_ok=True)

        # Storage limits
        self.max_raw_images = storage_config['max_raw_images']
        self.max_annotated_images = storage_config['max_annotated_images']
        self.save_only_with_objects = storage_config['save_only_with_objects']
        self.min_objects = storage_config['min_objects']

        # Image format
        self.format = storage_config['format']
        self.quality = storage_config['quality']

        # Initialize database
        self.db_path = storage_config['db_path']
        self._init_database()

        # CV Bridge
        self.bridge = CvBridge()

        # Cache for latest image and detections
        self.latest_image = None
        self.latest_detections = None

        # QoS profiles
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        detection_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
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
            detection_qos
        )

        self.get_logger().info(f'Image storage initialized at {self.base_path}')

    def _default_config(self):
        """Return default configuration."""
        return {
            'storage': {
                'base_path': '/home/robot/wheeltec_data/images',
                'raw_subdir': 'raw',
                'annotated_subdir': 'annotated',
                'max_raw_images': 1000,
                'max_annotated_images': 1000,
                'save_only_with_objects': True,
                'min_objects': 1,
                'format': 'jpg',
                'quality': 85,
                'db_path': '/home/robot/wheeltec_data/images/metadata.db',
                'topics': {
                    'camera_input': '/camera/color/image_raw',
                    'detections_input': '/detections'
                }
            }
        }

    def _init_database(self):
        """Initialize SQLite database for metadata."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS images (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                frame_id TEXT NOT NULL,
                raw_path TEXT NOT NULL,
                annotated_path TEXT,
                object_count INTEGER NOT NULL,
                classes TEXT,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        conn.commit()
        conn.close()

    def image_callback(self, msg: Image):
        """Cache latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def detection_callback(self, msg: Detection2DArray):
        """Process detections and save images."""
        if self.latest_image is None:
            return

        try:
            num_objects = len(msg.detections)

            # Check if we should save
            if self.save_only_with_objects and num_objects < self.min_objects:
                return

            # Generate filenames
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            frame_id = msg.header.frame_id or 'camera'

            raw_filename = f'{timestamp}_{frame_id}_raw.{self.format}'
            annotated_filename = f'{timestamp}_{frame_id}_annotated.{self.format}'

            raw_filepath = self.raw_path / raw_filename
            annotated_filepath = self.annotated_path / annotated_filename

            # Save raw image
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            cv2.imwrite(str(raw_filepath), self.latest_image, encode_params)

            # Create annotated image
            annotated_image = self.latest_image.copy()
            classes = []

            for det in msg.detections:
                # Draw bounding box
                x_min, y_min = det.x_min, det.y_min
                x_max, y_max = det.x_max, det.y_max

                # Color based on class
                if det.class_name == 'person':
                    color = (0, 255, 0)  # Green
                elif det.class_name in ['car', 'truck', 'bus', 'motorcycle', 'bicycle']:
                    color = (255, 0, 0)  # Blue
                else:
                    color = (0, 255, 255)  # Yellow

                cv2.rectangle(annotated_image, (x_min, y_min), (x_max, y_max), color, 2)

                # Draw label
                label = f'{det.class_name} {det.confidence:.2f}'
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_image, (x_min, y_min - label_h - 5),
                            (x_min + label_w, y_min), color, -1)
                cv2.putText(annotated_image, label, (x_min, y_min - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                classes.append(det.class_name)

            # Save annotated image
            cv2.imwrite(str(annotated_filepath), annotated_image, encode_params)

            # Save metadata to database
            self._save_metadata(timestamp, frame_id, str(raw_filepath),
                              str(annotated_filepath), num_objects, ','.join(classes))

            # Enforce storage limits
            self._enforce_limits()

            self.get_logger().info(f'Saved image with {num_objects} objects')

        except Exception as e:
            self.get_logger().error(f'Storage error: {e}')

    def _save_metadata(self, timestamp: str, frame_id: str, raw_path: str,
                      annotated_path: str, object_count: int, classes: str):
        """Save image metadata to database."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute('''
            INSERT INTO images (timestamp, frame_id, raw_path, annotated_path, object_count, classes)
            VALUES (?, ?, ?, ?, ?, ?)
        ''', (timestamp, frame_id, raw_path, annotated_path, object_count, classes))

        conn.commit()
        conn.close()

    def _enforce_limits(self):
        """Enforce FIFO storage limits."""
        # Check raw images
        raw_images = sorted(self.raw_path.glob(f'*.{self.format}'))
        if len(raw_images) > self.max_raw_images:
            # Delete oldest
            for img in raw_images[:len(raw_images) - self.max_raw_images]:
                img.unlink()
                self.get_logger().info(f'Deleted old raw image: {img.name}')

        # Check annotated images
        annotated_images = sorted(self.annotated_path.glob(f'*.{self.format}'))
        if len(annotated_images) > self.max_annotated_images:
            # Delete oldest
            for img in annotated_images[:len(annotated_images) - self.max_annotated_images]:
                img.unlink()
                self.get_logger().info(f'Deleted old annotated image: {img.name}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageStorageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
