import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pid_dir = get_package_share_directory('wheeltec_pid_controller')
    pid_params = os.path.join(pid_dir, 'config', 'pid_params.yaml')

    return LaunchDescription([
        Node(
            package='wheeltec_pid_controller',
            executable='pid_controller_node',
            name='pid_controller',
            parameters=[pid_params],
            remappings=[
                ('cmd_vel_raw', '/cmd_vel_raw'),
                ('cmd_vel', '/cmd_vel'),
            ],
            output='screen',
        ),
    ])
