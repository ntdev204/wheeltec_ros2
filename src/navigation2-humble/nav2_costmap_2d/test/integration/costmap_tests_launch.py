

import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    launchFile = os.path.join(os.getenv('TEST_LAUNCH_DIR'), 'costmap_map_server.launch.py')
    testExecutable = os.getenv('TEST_EXECUTABLE')

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server']}, {'autostart': True}])

    ld = LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([launchFile])),
        lifecycle_manager
    ])

    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name='costmap_tests',
        output='screen'
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
