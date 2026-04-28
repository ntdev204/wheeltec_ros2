import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    wheeltec_twist_mux_dir = get_package_share_directory('wheeltec_twist_mux')
    wheeltec_scada_bridge_dir = get_package_share_directory('wheeltec_scada_bridge')
    wheeltec_launch_dir = get_package_share_directory('turn_on_wheeltec_robot')

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
        lidar_serial_reset,

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

        TimerAction(
            period=6.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(wheeltec_twist_mux_dir, 'launch', 'twist_mux.launch.py')
                    )
                ),
            ]
        ),

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
