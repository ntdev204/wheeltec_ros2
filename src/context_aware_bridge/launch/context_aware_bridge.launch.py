from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='25.12.4.100',
        description='IP address of the Jetson AI server',
    )
    sensor_ingest_port_arg = DeclareLaunchArgument(
        'sensor_ingest_port',
        default_value='5555',
        description='ZMQ PUSH port for sensor protobuf messages to Jetson',
    )
    result_publish_port_arg = DeclareLaunchArgument(
        'result_publish_port',
        default_value='5556',
        description='ZMQ PUB port for adaptive perception results from Jetson',
    )
    nav_cmd_port_arg = DeclareLaunchArgument(
        'nav_cmd_port',
        default_value='9091',
        description='TCP port for adaptive NAV_CMD packets from Jetson',
    )
    heartbeat_port_arg = DeclareLaunchArgument(
        'heartbeat_port',
        default_value='9093',
        description='TCP port for adaptive heartbeat packets from Jetson',
    )

    bridge_node = Node(
        package='context_aware_bridge',
        executable='context_aware_bridge',
        name='context_aware_bridge',
        output='screen',
        parameters=[{
            'jetson_ip':        LaunchConfiguration('jetson_ip'),
            'sensor_ingest_port': LaunchConfiguration('sensor_ingest_port'),
            'result_publish_port': LaunchConfiguration('result_publish_port'),
            'nav_cmd_port': LaunchConfiguration('nav_cmd_port'),
            'heartbeat_port': LaunchConfiguration('heartbeat_port'),
        }],
    )

    return LaunchDescription([
        jetson_ip_arg,
        sensor_ingest_port_arg,
        result_publish_port_arg,
        nav_cmd_port_arg,
        heartbeat_port_arg,
        bridge_node,
    ])
