

import os

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    mapFile = os.getenv('TEST_MAP')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': mapFile}])
    ])
