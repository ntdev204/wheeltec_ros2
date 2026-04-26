import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_twist_mux_dir = get_package_share_directory('wheeltec_twist_mux')
    wheeltec_scada_bridge_dir = get_package_share_directory('wheeltec_scada_bridge')
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
                os.path.join(wheeltec_nav_dir, 'launch', 'wheeltec_nav2.launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'use_sim_time': 'false',
            }.items(),
        ),

        # 3. Twist Mux
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wheeltec_twist_mux_dir, 'launch', 'twist_mux.launch.py')
            )
        ),

        # 4. SCADA ZMQ Bridge (ROS2 <-> Web Server)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wheeltec_scada_bridge_dir, 'launch', 'scada_bridge.launch.py')
            )
        ),
    ])
