# SAU (simplified, chỉ giữ lại lidar đang dùng):
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    Lslidar_dir = get_package_share_directory('lslidar_driver')
    Lslidar_launch_dir = os.path.join(Lslidar_dir, 'launch')

    Lsn10p = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(Lslidar_launch_dir, 'lsn10p_launch.py')),)

    ld = LaunchDescription()
    ld.add_action(Lsn10p)
    return ld