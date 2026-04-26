import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    twist_mux_cfg = os.path.join(
        get_package_share_directory('wheeltec_twist_mux'),
        'config',
        'twist_mux.yaml',
    )

    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='192.168.1.100',
        description='IP address of the Jetson AI server',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg],
        remappings=[
            ('cmd_vel_out', '/cmd_vel'),
        ],
    )

    context_bridge_node = Node(
        package='context_aware_bridge',
        executable='context_aware_bridge',
        name='context_aware_bridge',
        output='screen',
        parameters=[{
            'jetson_ip':        LaunchConfiguration('jetson_ip'),
            'nav_cmd_port':     5555,
            'robot_state_port': 5560,
        }],
    )

    return LaunchDescription([
        jetson_ip_arg,
        twist_mux_node,
        context_bridge_node,
    ])
