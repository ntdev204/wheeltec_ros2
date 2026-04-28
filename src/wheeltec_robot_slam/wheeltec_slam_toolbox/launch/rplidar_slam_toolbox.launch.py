import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_dir = get_package_share_directory('wheeltec_slam_toolbox')

    return LaunchDescription([
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
