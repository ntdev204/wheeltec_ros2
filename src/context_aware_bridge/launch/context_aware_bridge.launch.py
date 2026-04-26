from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='192.168.1.100',
        description='IP address of the Jetson AI server',
    )
    nav_cmd_port_arg = DeclareLaunchArgument(
        'nav_cmd_port',
        default_value='5555',
        description='ZMQ port for NavigationCommand from Jetson',
    )
    robot_state_port_arg = DeclareLaunchArgument(
        'robot_state_port',
        default_value='5560',
        description='ZMQ port for RobotState sent to Jetson',
    )

    bridge_node = Node(
        package='context_aware_bridge',
        executable='context_aware_bridge',
        name='context_aware_bridge',
        output='screen',
        parameters=[{
            'jetson_ip':        LaunchConfiguration('jetson_ip'),
            'nav_cmd_port':     LaunchConfiguration('nav_cmd_port'),
            'robot_state_port': LaunchConfiguration('robot_state_port'),
        }],
    )

    return LaunchDescription([
        jetson_ip_arg,
        nav_cmd_port_arg,
        robot_state_port_arg,
        bridge_node,
    ])
