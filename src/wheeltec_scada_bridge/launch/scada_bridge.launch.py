from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeltec_scada_bridge',
            executable='scada_bridge',
            name='scada_control_node',
            output='screen',
            parameters=[{
                'zmq_cmd_port': 5555,
                'zmq_telemetry_port': 5556,
                'zmq_camera_port': 5557,
                'camera_topic': '/camera/color/image_raw',
                'slam_start_cmd': 'ros2 launch wheeltec_slam_toolbox slam_only.launch.py',
                'nav2_start_cmd': 'ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py map:={map}',
                'map_saver_cmd': 'ros2 run nav2_map_server map_saver_cli -f {map_prefix}',
                'map_save_dir': '/home/rai/wheeltec_ros2/data/map',
            }],
        ),
    ])
