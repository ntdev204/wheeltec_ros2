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

    adaptive_host_arg = DeclareLaunchArgument(
        'adaptive_host',
        default_value='',
        description='IP address or hostname of the laptop adaptive runtime',
    )
    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='',
        description='Deprecated alias for adaptive_host',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg],
        remappings=[
            ('cmd_vel_out', '/cmd_vel'),  # direct to robot base, no safety relay
        ],
    )

    context_bridge_node = Node(
        package='context_aware_bridge',
        executable='context_aware_bridge',
        name='context_aware_bridge',
        output='screen',
        parameters=[{
            'adaptive_host':    LaunchConfiguration('adaptive_host'),
            'jetson_ip':        LaunchConfiguration('jetson_ip'),
            'raspi_ip':         '25.12.4.101',
            'sensor_ingest_port': 5555,
            'result_publish_port': 5556,
            'nav_cmd_port': 9091,
            'heartbeat_port': 9093,
        }],
    )

    return LaunchDescription([
        adaptive_host_arg,
        jetson_ip_arg,
        twist_mux_node,
        context_bridge_node,
    ])
