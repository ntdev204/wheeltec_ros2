import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    wheeltec_twist_mux_dir = get_package_share_directory('wheeltec_twist_mux')
    wheeltec_scada_bridge_dir = get_package_share_directory('wheeltec_scada_bridge')
    wheeltec_launch_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_slam_dir = get_package_share_directory('wheeltec_slam_toolbox')

    # DTR toggle để reset Lidar — tương đương rút cắm USB.
    # Đáng tin cậy hơn gửi lệnh protocol vì không bị ảnh hưởng bởi trạng thái device.
    lidar_serial_reset = ExecuteProcess(
        cmd=['bash', '-c',
             'python3 -c "'
             'import serial, time; '
             's = serial.Serial(\'/dev/wheeltec_lidar\', 115200, timeout=1); '
             's.setDTR(False); s.setRTS(False); '
             'time.sleep(0.5); '
             's.setDTR(True); s.setRTS(True); '
             'time.sleep(0.2); '
             's.close(); '
             'print(chr(10) + \'[lidar_reset] DTR toggle OK — waiting for motor spin-up...\')'
             '" || echo "[lidar_reset] Lidar reset skipped (port not ready)"'],
        output='screen',
    )

    return LaunchDescription([
        # 0. DTR toggle Lidar (tương đương unplug/replug, tránh lỗi 80008002)
        lidar_serial_reset,

        # 1. Base Hardware Layer (Chassis, Lidar, IMU, EKF, TF)
        #    Delay 5s để motor đủ thời gian spin-up sau DTR reset
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_launch_dir, 'launch', 'wheeltec_sensors.launch.py')
                    )
                ),
            ]
        ),

        # 1.5. Filter raw lidar scans once for all runtime modes.
        # SLAM, AMCL and Nav2 costmaps should consume /scan_filtered.
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='wheeltec_slam_toolbox',
                    executable='scan_box_filter_node',
                    name='scan_box_filter_node',
                    output='screen',
                    parameters=[
                        os.path.join(wheeltec_slam_dir, 'config', 'laser_filter.yaml')
                    ],
                ),
            ]
        ),

        # 2. Twist Mux + Safety Shield
        TimerAction(
            period=6.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_twist_mux_dir, 'launch', 'twist_mux.launch.py')
                    ),
                ),
            ]
        ),

        # 3. SCADA ZMQ Bridge (ROS2 <-> Web Server)
        # NOTE: Nav2 đã được tắt — chế độ bám người KHÔNG cần map/global_costmap
        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_scada_bridge_dir, 'launch', 'scada_bridge.launch.py')
                    )
                ),
            ]
        ),
    ])
