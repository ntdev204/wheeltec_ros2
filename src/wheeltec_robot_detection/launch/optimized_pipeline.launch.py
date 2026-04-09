"""Optimized launch file: detection + tracking + WebSocket streaming (no ZMQ)."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheeltec_robot_detection')

    detection_config = os.path.join(pkg_dir, 'config', 'detection_params.yaml')
    tracker_config = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')
    streaming_config = os.path.join(pkg_dir, 'config', 'streaming_params.yaml')

    return LaunchDescription([
        # Detection node (TensorRT inference)
        Node(
            package='wheeltec_robot_detection',
            executable='detection_node',
            name='detection_node',
            parameters=[{'config_file': detection_config}],
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
        ),

        # Tracker node
        Node(
            package='wheeltec_robot_detection',
            executable='tracker_node',
            name='tracker_node',
            parameters=[{'config_file': tracker_config}],
            output='screen'
        ),

        # WebSocket streaming (replaces ZMQ video_stream_node)
        Node(
            package='wheeltec_robot_detection',
            executable='websocket_stream_node',
            name='websocket_stream_node',
            parameters=[{'config_file': streaming_config}],
            output='screen'
        ),

        # Dynamic obstacle publisher
        Node(
            package='wheeltec_robot_detection',
            executable='dynamic_obstacle_publisher',
            name='dynamic_obstacle_publisher',
            output='screen'
        ),
    ])
