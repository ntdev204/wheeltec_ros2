#!/usr/bin/env python3
"""Dynamic obstacle publisher - converts tracked objects to dynamic obstacles for Nav2."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Vector3
import math

from wheeltec_robot_msg.msg import TrackedObjectArray, DynamicObstacle, DynamicObstacleArray


class DynamicObstaclePublisher(Node):
    """Convert tracked objects to dynamic obstacles with trajectory prediction."""

    def __init__(self):
        super().__init__('dynamic_obstacle_publisher')

        # Declare parameters
        self.declare_parameter('velocity_threshold', 0.5)  # pixels/frame
        self.declare_parameter('prediction_time', 1.0)  # seconds
        self.declare_parameter('camera_height', 1.0)  # meters
        self.declare_parameter('pixel_to_meter_ratio', 0.01)  # meters per pixel

        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.prediction_time = self.get_parameter('prediction_time').value
        self.camera_height = self.get_parameter('camera_height').value
        self.pixel_to_meter = self.get_parameter('pixel_to_meter_ratio').value

        # QoS profiles
        tracking_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.tracked_objects_sub = self.create_subscription(
            TrackedObjectArray,
            '/tracked_objects',
            self.tracked_objects_callback,
            tracking_qos
        )

        # Publishers
        self.dynamic_obstacles_pub = self.create_publisher(
            DynamicObstacleArray,
            '/dynamic_obstacles',
            tracking_qos
        )

        self.get_logger().info('Dynamic obstacle publisher initialized')

    def tracked_objects_callback(self, msg: TrackedObjectArray):
        """Convert tracked objects to dynamic obstacles."""
        try:
            obstacles_msg = DynamicObstacleArray()
            obstacles_msg.header = msg.header

            for obj in msg.objects:
                # Calculate velocity magnitude
                velocity_mag = math.sqrt(obj.velocity_x**2 + obj.velocity_y**2)

                # Only publish moving objects
                if velocity_mag < self.velocity_threshold:
                    continue

                obstacle = DynamicObstacle()
                obstacle.header = msg.header
                obstacle.track_id = obj.track_id
                obstacle.class_name = obj.class_name

                # Convert pixel coordinates to map frame (simplified)
                # In real implementation, use camera calibration and TF transforms
                center_x = (obj.x_min + obj.x_max) / 2.0
                center_y = (obj.y_min + obj.y_max) / 2.0

                # Position in map frame (placeholder - needs proper transformation)
                obstacle.position = Point()
                obstacle.position.x = center_x * self.pixel_to_meter
                obstacle.position.y = center_y * self.pixel_to_meter
                obstacle.position.z = 0.0

                # Velocity in map frame
                obstacle.velocity = Vector3()
                obstacle.velocity.x = obj.velocity_x * self.pixel_to_meter
                obstacle.velocity.y = obj.velocity_y * self.pixel_to_meter
                obstacle.velocity.z = 0.0

                # Predict position after prediction_time
                obstacle.predicted_position = Vector3()
                obstacle.predicted_position.x = obstacle.position.x + obstacle.velocity.x * self.prediction_time
                obstacle.predicted_position.y = obstacle.position.y + obstacle.velocity.y * self.prediction_time
                obstacle.predicted_position.z = 0.0

                # Estimate obstacle radius based on bbox size
                width = (obj.x_max - obj.x_min) * self.pixel_to_meter
                height = (obj.y_max - obj.y_min) * self.pixel_to_meter
                obstacle.radius = max(width, height) / 2.0

                obstacle.confidence = obj.confidence

                obstacles_msg.obstacles.append(obstacle)

            # Publish
            if len(obstacles_msg.obstacles) > 0:
                self.dynamic_obstacles_pub.publish(obstacles_msg)
                self.get_logger().info(f'Published {len(obstacles_msg.obstacles)} dynamic obstacles')

        except Exception as e:
            self.get_logger().error(f'Dynamic obstacle conversion error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
