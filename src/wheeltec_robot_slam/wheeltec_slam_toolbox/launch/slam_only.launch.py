
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    params_file = (
        get_package_share_directory('wheeltec_slam_toolbox')
        + '/config/mapper_params_online_sync.yaml'
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],
        )
    ])
