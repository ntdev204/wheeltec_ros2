"""Launch file for dynamic obstacle avoidance on Raspberry Pi."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeltec_dynamic_avoidance',
            executable='obstacle_avoider_node',
            name='obstacle_avoider_node',
            parameters=[
                {'costmap_resolution': 0.05},
                {'costmap_width': 10.0},
                {'costmap_height': 10.0},
                {'inflation_radius': 0.5},
                {'update_frequency': 10.0}
            ],
            output='screen'
        ),
    ])
