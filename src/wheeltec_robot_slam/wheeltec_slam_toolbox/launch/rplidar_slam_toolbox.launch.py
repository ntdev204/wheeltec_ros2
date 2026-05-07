#!/usr/bin/env python3
"""
rplidar_slam_toolbox.launch.py - filtered RPLidar SLAM (async mode)

File nay chay laser scan filter + SLAM Toolbox (async).
Robot base + lidar da duoc khoi dong boi wheeltec_sensors.launch.py.

Su dung doc lap:
  ros2 launch wheeltec_slam_toolbox rplidar_slam_toolbox.launch.py

Hoac tich hop vao bringup tong (sau wheeltec_sensors):
  them vao prod_bringup.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_dir = get_package_share_directory('wheeltec_slam_toolbox')

    return LaunchDescription([
        Node(
            package='wheeltec_slam_toolbox',
            executable='scan_box_filter_node',
            name='scan_box_filter_node',
            output='screen',
            parameters=[
                os.path.join(slam_dir, 'config', 'laser_filter.yaml')
            ],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(slam_dir, 'config', 'mapper_params_rplidar_a1.yaml')
            ],
            remappings=[('odom', 'odom_combined')],
        ),
    ])
