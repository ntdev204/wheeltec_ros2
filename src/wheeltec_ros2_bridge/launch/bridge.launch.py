import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeltec_ros2_bridge',
            executable='bridge_node',
            name='context_aware_bridge',
            output='screen',
            parameters=[
                {'jetson_ip': '25.12.4.100'}
            ]
        )
    ])
