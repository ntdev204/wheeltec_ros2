import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  rviz2_config = os.path.join(
      get_package_share_directory('ldlidar_sl_ros2'),
      'rviz2',
      'ldlidar.rviz'
  )
  
  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_show_ld14',
      arguments=['-d',rviz2_config],
      output='screen'
  )

  ldlidar_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('ldlidar_sl_ros2'),
          '/launch/ld14.launch.py'
      ])
  )

  ld = LaunchDescription()

  ld.add_action(ldlidar_launch)
  ld.add_action(rviz2_node)
  
  return ld