#!/usr/bin/env python3
"""ByteTrack tracking node for multi-object tracking."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from pathlib import Path
import yaml

from wheeltec_robot_msg.msg import (
    Detection2DArray, TrackedObject, TrackedObjectArray,
    TrackedHuman, TrackedHumanArray
)
from .tracking import ByteTracker


class TrackerNode(Node):
    """ROS2 node for multi-object tracking using ByteTrack."""

    def __init__(self):
        super().__init__('tracker_node')

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

        tracker_config = self.config['tracker']
        topics_config = self.config['topics']

        # Initialize tracker
        self.tracker = ByteTracker(
            track_thresh=tracker_config['track_thresh'],
            track_buffer=tracker_config['track_buffer'],
            match_thresh=tracker_config['match_thresh'],
            min_box_area=tracker_config['min_box_area']
        )

        self.human_class_id = tracker_config['human_class_id']

        # QoS profiles
        detection_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        tracking_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            topics_config['detections_input'],
            self.detection_callback,
            detection_qos
        )

        # Publishers
        self.tracked_objects_pub = self.create_publisher(
            TrackedObjectArray,
            topics_config['tracked_objects_output'],
            tracking_qos
        )

        self.tracked_humans_pub = self.create_publisher(
            TrackedHumanArray,
            topics_config['tracked_humans_output'],
            tracking_qos
        )

        self.get_logger().info('Tracker node initialized')

    def _default_config(self):
        """Return default configuration."""
        return {
            'tracker': {
                'track_thresh': 0.5,
                'track_buffer': 30,
                'match_thresh': 0.8,
                'min_box_area': 100,
                'human_class_id': 0
            },
            'topics': {
                'detections_input': '/detections',
                'tracked_objects_output': '/tracked_objects',
                'tracked_humans_output': '/tracked_humans'
            }
        }

    def detection_callback(self, msg: Detection2DArray):
        """Process detections and update tracks."""
        try:
            # Convert ROS detections to tracker format
            detections = []
            for det in msg.detections:
                detections.append({
                    'class_id': det.class_id,
                    'class_name': det.class_name,
                    'confidence': det.confidence,
                    'bbox': [det.x_min, det.y_min, det.x_max, det.y_max]
                })

            # Update tracker
            tracks = self.tracker.update(detections)

            # Publish tracked objects
            tracked_objects_msg = TrackedObjectArray()
            tracked_objects_msg.header = msg.header

            tracked_humans_msg = TrackedHumanArray()
            tracked_humans_msg.header = msg.header

            for track in tracks:
                bbox = track.get_bbox()
                vx, vy = track.get_velocity()

                # Create tracked object message
                obj_msg = TrackedObject()
                obj_msg.header = msg.header
                obj_msg.track_id = track.track_id
                obj_msg.class_name = track.class_name
                obj_msg.class_id = track.class_id
                obj_msg.confidence = track.confidence
                obj_msg.x_min = int(bbox[0])
                obj_msg.y_min = int(bbox[1])
                obj_msg.x_max = int(bbox[2])
                obj_msg.y_max = int(bbox[3])
                obj_msg.velocity_x = vx
                obj_msg.velocity_y = vy
                obj_msg.age = track.age
                obj_msg.is_confirmed = track.is_confirmed

                tracked_objects_msg.objects.append(obj_msg)

                # If human, also add to humans array
                if track.class_id == self.human_class_id:
                    human_msg = TrackedHuman()
                    human_msg.header = msg.header
                    human_msg.track_id = track.track_id
                    human_msg.confidence = track.confidence
                    human_msg.x_min = int(bbox[0])
                    human_msg.y_min = int(bbox[1])
                    human_msg.x_max = int(bbox[2])
                    human_msg.y_max = int(bbox[3])
                    human_msg.velocity_x = vx
                    human_msg.velocity_y = vy
                    human_msg.age = track.age
                    human_msg.is_confirmed = track.is_confirmed

                    tracked_humans_msg.humans.append(human_msg)

            # Publish
            self.tracked_objects_pub.publish(tracked_objects_msg)
            self.tracked_humans_pub.publish(tracked_humans_msg)

            if len(tracks) > 0:
                self.get_logger().info(
                    f'Tracking {len(tracks)} objects ({len(tracked_humans_msg.humans)} humans)'
                )

        except Exception as e:
            self.get_logger().error(f'Tracking error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
