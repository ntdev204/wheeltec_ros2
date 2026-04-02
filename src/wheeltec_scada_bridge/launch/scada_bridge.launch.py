from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeltec_scada_bridge',
            executable='scada_bridge',
            name='scada_control_node',
            output='screen',
            parameters=[{
                'zmq_cmd_port': 5555,
                'zmq_telemetry_port': 5556,
                'zmq_camera_port': 5557,
                'camera_topic': '/camera/color/image_raw',
            }],
        ),
    ])
