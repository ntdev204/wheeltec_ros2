import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    wheeltec_twist_mux_dir = get_package_share_directory('wheeltec_twist_mux')
    wheeltec_scada_bridge_dir = get_package_share_directory('wheeltec_scada_bridge')
    wheeltec_launch_dir = get_package_share_directory('turn_on_wheeltec_robot')

    # Gửi lệnh RESET (0xA5 0x40) để reboot firmware Lidar.
    # Dùng RESET thay vì STOP vì STOP sẽ tắt motor, khiến rplidar_node
    # không thể start scan (lỗi 80008002). RESET khởi động lại toàn bộ firmware.
    lidar_serial_reset = ExecuteProcess(
        cmd=['bash', '-c',
             'python3 -c "'
             'import serial, time; '
             's = serial.Serial(\'/dev/wheeltec_lidar\', 115200, timeout=1); '
             's.write(bytes([0xa5, 0x40])); '
             'time.sleep(0.1); '
             's.close(); '
             'print(chr(10) + \'[lidar_reset] Lidar RESET sent, waiting for motor spin-up...\')'
             '" || echo "[lidar_reset] Lidar reset skipped (port not ready)"'],
        output='screen',
    )

    return LaunchDescription([
        # 0. Reset Lidar serial trước khi launch (tránh Timeout khi restart)
        lidar_serial_reset,

        # 1. Base Hardware Layer (Chassis, Lidar, IMU, EKF, TF)
        #    Delay 1.5s để đảm bảo serial reset hoàn tất trước khi rplidar_node khởi động
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_launch_dir, 'launch', 'wheeltec_sensors.launch.py')
                    )
                ),
            ]
        ),

        # 2. Twist Mux + Safety Shield
        TimerAction(
            period=2.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_twist_mux_dir, 'launch', 'twist_mux.launch.py')
                    )
                ),
            ]
        ),

        # 3. SCADA ZMQ Bridge (ROS2 <-> Web Server)
        # NOTE: Nav2 đã được tắt — chế độ bám người KHÔNG cần map/global_costmap
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_scada_bridge_dir, 'launch', 'scada_bridge.launch.py')
                    )
                ),
            ]
        ),
    ])
