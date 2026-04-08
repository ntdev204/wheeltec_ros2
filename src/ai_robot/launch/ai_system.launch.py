from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turn_on_camera_ai'),
                'launch', 'wheeltec_camera.launch.py'
            )
        )
    )

    detection = Node(
        package='ai_robot',
        executable='detection_node',
        name='detector',
        output='screen',
    )

    tracking = Node(
        package='ai_robot',
        executable='tracking_node',
        name='tracker',
        output='screen',
    )

    context = Node(
        package='ai_robot',
        executable='context_node',
        name='context',
        output='screen',
    )

    logger = Node(
        package='ai_robot',
        executable='data_logger_node',
        name='logger',
        output='screen',
    )

    return LaunchDescription([
        camera,
        detection,
        tracking,
        context,
        logger,
    ])
