"""Launch file for full AI pipeline: detection + tracking + storage + streaming."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheeltec_robot_detection')

    # Configuration files
    detection_config = os.path.join(pkg_dir, 'config', 'detection_params.yaml')
    tracker_config = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')
    storage_config = os.path.join(pkg_dir, 'config', 'storage_params.yaml')
    streaming_config = os.path.join(pkg_dir, 'config', 'streaming_params.yaml')

    return LaunchDescription([
        # Detection node
        Node(
            package='wheeltec_robot_detection',
            executable='detection_node',
            name='detection_node',
            parameters=[{'config_file': detection_config}],
            output='screen'
        ),

        # Tracker node
        Node(
            package='wheeltec_robot_detection',
            executable='tracker_node',
            name='tracker_node',
            parameters=[{'config_file': tracker_config}],
            output='screen'
        ),

        # Image storage node
        Node(
            package='wheeltec_robot_detection',
            executable='image_storage_node',
            name='image_storage_node',
            parameters=[{'config_file': storage_config}],
            output='screen'
        ),

        # Video streaming node
        Node(
            package='wheeltec_robot_detection',
            executable='video_stream_node',
            name='video_stream_node',
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
