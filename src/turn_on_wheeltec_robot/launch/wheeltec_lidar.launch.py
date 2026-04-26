#!/usr/bin/env python3
"""
wheeltec_lidar.launch.py - RPLidar A1M8
Thay the LSlidar trong wheeltec_sensors.launch.py

udev symlink: /dev/wheeltec_lidar -> ttyUSB0 (CP2102, serial=0001)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        # RPLidar A1M8 node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port':      '/dev/wheeltec_lidar',
                'serial_baudrate':  115200,
                'frame_id':         'laser',
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }],
            output='screen',
        ),

        # Static TF: laser_link (URDF mini_mec) <-> laser (RPLidar frame_id)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_laser_link',
            arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'laser'],
        ),
    ])
