"""
slam_only.launch.py
────────────────────
Chỉ khởi động sync_slam_toolbox_node.
Robot driver và lidar đã được scada_bridge quản lý riêng,
KHÔNG include lại turn_on_wheeltec_robot hay wheeltec_lidar ở đây.

Dùng bởi wheeltec_scada_bridge khi người dùng chọn Mapping trên web UI:
  ros2 launch wheeltec_slam_toolbox slam_only.launch.py

NOTE về odom topic:
  - wheeltec_robot driver publish lên /odom_combined (robot dạng omni/mec)
    hoặc /odom (robot dạng diff). SLAM Toolbox config dùng odom_frame:
    odom_combined nên KHÔNG cần remap — SLAM Toolbox sẽ tự subscribe
    /odom_combined qua odom_frame param (TF-based, không phải topic sub).
  - Scan topic: /scan (mặc định từ lidar driver).
"""

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
            # Không remap odom ở đây — SLAM Toolbox dùng TF (odom_frame param)
            # chứ không subscribe trực tiếp /odom topic.
        )
    ])
