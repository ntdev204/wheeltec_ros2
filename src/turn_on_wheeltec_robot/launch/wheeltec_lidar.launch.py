import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port':      '/dev/wheeltec_lidar',
                'serial_baudrate':  115200,
                'frame_id':         'laser',
                'angle_compensate': True,
            }],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_laser_link',
            arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'laser'],
        ),
    ])
