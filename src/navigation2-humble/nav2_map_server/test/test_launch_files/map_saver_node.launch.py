

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions


def generate_launch_description():
    map_publisher = f"{os.path.dirname(os.getenv('TEST_EXECUTABLE'))}/test_map_saver_publisher"

    ld = LaunchDescription()

    map_saver_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[os.path.join(os.getenv('TEST_DIR'),
                    'map_saver_params.yaml')])

    map_publisher_cmd = ExecuteProcess(
        cmd=[map_publisher])

    ld.add_action(map_saver_server_cmd)
    ld.add_action(map_publisher_cmd)

    return ld
