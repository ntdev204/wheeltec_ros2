import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_launch_dir = get_package_share_directory('turn_on_wheeltec_robot')

    return LaunchDescription([
        # 1. Base Hardware Layer (Chassis, Lidar, Camera, IMU, EKF, TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wheeltec_launch_dir, 'launch', 'wheeltec_sensors.launch.py')
            )
        ),

        # 2. Application Layer (Nav2, AMCL, Costmaps)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('wheeltec_robot_nav2'), 'launch', 'wheeltec_nav2.launch.py')
            )
        ),

        # 3. SCADA ZMQ Bridge (ROS2 <-> Web Server)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('wheeltec_scada_bridge'), 'launch', 'scada_bridge.launch.py')
            )
        ),
    ])
