#!/usr/bin/env python3
"""Dynamic obstacle avoider node for Nav2 integration."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

from wheeltec_robot_msg.msg import DynamicObstacleArray


class ObstacleAvoiderNode(Node):
    """ROS2 node for dynamic obstacle avoidance with Nav2 integration."""

    def __init__(self):
        super().__init__('obstacle_avoider_node')

        # Declare parameters
        self.declare_parameter('costmap_resolution', 0.05)  # meters per cell
        self.declare_parameter('costmap_width', 10.0)  # meters
        self.declare_parameter('costmap_height', 10.0)  # meters
        self.declare_parameter('inflation_radius', 0.5)  # meters
        self.declare_parameter('update_frequency', 10.0)  # Hz

        self.resolution = self.get_parameter('costmap_resolution').value
        self.width_m = self.get_parameter('costmap_width').value
        self.height_m = self.get_parameter('costmap_height').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        update_freq = self.get_parameter('update_frequency').value

        # Calculate grid dimensions
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)

        # Initialize costmap
        self.costmap = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)

        # Cache for dynamic obstacles
        self.dynamic_obstacles = None

        # QoS profiles
        obstacle_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.obstacles_sub = self.create_subscription(
            DynamicObstacleArray,
            '/dynamic_obstacles',
            self.obstacles_callback,
            obstacle_qos
        )

        # Publishers
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/dynamic_costmap',
            10
        )

        # Timer for costmap updates
        self.create_timer(1.0 / update_freq, self.update_costmap)

        self.get_logger().info('Obstacle avoider node initialized')

    def obstacles_callback(self, msg: DynamicObstacleArray):
        """Cache dynamic obstacles."""
        self.dynamic_obstacles = msg

    def update_costmap(self):
        """Update and publish dynamic costmap."""
        if self.dynamic_obstacles is None:
            return

        try:
            # Reset costmap
            self.costmap.fill(0)

            # Add obstacles to costmap
            for obstacle in self.dynamic_obstacles.obstacles:
                # Current position
                self._add_obstacle_to_costmap(
                    obstacle.position.x,
                    obstacle.position.y,
                    obstacle.radius,
                    cost=100
                )

                # Predicted position
                self._add_obstacle_to_costmap(
                    obstacle.predicted_position.x,
                    obstacle.predicted_position.y,
                    obstacle.radius,
                    cost=80
                )

                # Trajectory path (interpolate between current and predicted)
                steps = 5
                for i in range(1, steps):
                    t = i / steps
                    x = obstacle.position.x + t * (obstacle.predicted_position.x - obstacle.position.x)
                    y = obstacle.position.y + t * (obstacle.predicted_position.y - obstacle.position.y)
                    self._add_obstacle_to_costmap(x, y, obstacle.radius, cost=60)

            # Publish costmap
            costmap_msg = OccupancyGrid()
            costmap_msg.header.stamp = self.get_clock().now().to_msg()
            costmap_msg.header.frame_id = 'map'

            costmap_msg.info.resolution = self.resolution
            costmap_msg.info.width = self.width_cells
            costmap_msg.info.height = self.height_cells
            costmap_msg.info.origin.position.x = -self.width_m / 2.0
            costmap_msg.info.origin.position.y = -self.height_m / 2.0
            costmap_msg.info.origin.orientation.w = 1.0

            costmap_msg.data = self.costmap.flatten().tolist()

            self.costmap_pub.publish(costmap_msg)

        except Exception as e:
            self.get_logger().error(f'Costmap update error: {e}')

    def _add_obstacle_to_costmap(self, x: float, y: float, radius: float, cost: int):
        """Add obstacle to costmap with inflation."""
        # Convert world coordinates to grid coordinates
        grid_x = int((x - (-self.width_m / 2.0)) / self.resolution)
        grid_y = int((y - (-self.height_m / 2.0)) / self.resolution)

        # Calculate inflation radius in cells
        inflation_cells = int(self.inflation_radius / self.resolution)
        obstacle_cells = int(radius / self.resolution)

        # Add obstacle with inflation
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                gx = grid_x + dx
                gy = grid_y + dy

                # Check bounds
                if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                    # Calculate distance from obstacle center
                    dist = math.sqrt(dx**2 + dy**2) * self.resolution

                    # Set cost based on distance
                    if dist <= radius:
                        # Inside obstacle
                        self.costmap[gy, gx] = max(self.costmap[gy, gx], cost)
                    elif dist <= radius + self.inflation_radius:
                        # Inflation zone - linear decay
                        inflated_cost = int(cost * (1.0 - (dist - radius) / self.inflation_radius))
                        self.costmap[gy, gx] = max(self.costmap[gy, gx], inflated_cost)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
