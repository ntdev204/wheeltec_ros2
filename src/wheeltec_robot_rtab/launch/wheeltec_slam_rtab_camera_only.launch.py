"""
RTAB-Map SLAM using AstraS RGB-D camera ONLY (no LiDAR required).
Mapping strategy: Visual Odometry + RGB-D loop closure detection.

Usage:
  SLAM mode (build map):
    ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab_camera_only.launch.py

  Localization mode (use existing map):
    ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab_camera_only.launch.py localization:=true

AstraS topics used:
  /camera/color/image_raw      - RGB image
  /camera/color/camera_info    - Camera calibration
  /camera/depth/image_raw      - Depth image
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos          = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    rviz         = LaunchConfiguration('rviz')

    # ── Package directories ──────────────────────────────────────────────────
    robot_dir  = get_package_share_directory('turn_on_wheeltec_robot')
    astra_dir  = get_package_share_directory('astra_camera')

    # ── Included launch files ────────────────────────────────────────────────
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')
        ),
    )

    wheeltec_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(astra_dir, 'launch', 'astra.launch.xml')
        ),
    )

    # ── RTAB-Map parameters (RGB-D only, NO LiDAR scan) ─────────────────────
    rtabmap_parameters = {
        'frame_id':              'camera_link',
        'use_sim_time':          use_sim_time,

        # Subscribe ONLY to RGB-D, NOT to laser scan
        'subscribe_rgbd':        True,
        'subscribe_scan':        False,          # ← no LiDAR

        # Visual odometry (ICP disabled, use wheel/visual odom)
        'odom_sensor_sync':      False,
        'use_action_for_goal':   True,

        'qos_image':             qos,
        'qos_imu':               qos,

        # Strategy: F2M (feature-to-map) visual, no ICP
        'Reg/Strategy':          '0',            # 0=Visual, 1=ICP
        'Reg/Force3DoF':         'true',

        # Visual features & loop closure
        'Kp/MaxFeatures':        '500',
        'Vis/MinInliers':        '15',
        'RGBD/NeighborLinkRefining': 'True',
        'RGBD/AngularUpdate':    '0.01',
        'RGBD/LinearUpdate':     '0.01',
        'RGBD/OptimizeFromGraphEnd': 'False',

        # Mapping quality
        'Grid/FromDepth':        'true',         # build occupancy grid from depth
        'Grid/RangeMin':         '0.3',          # ignore very close points
        'Grid/RangeMax':         '4.0',          # AstraS max reliable depth ~4m
        'Grid/3D':               'false',        # 2D grid for navigation

        # Memory
        'Mem/NotLinkedNodesKept': 'false',
        'Optimizer/GravitySigma': '0',           # 2D mode, no gravity constraint
    }

    # ── Topic remappings for AstraS ──────────────────────────────────────────
    remappings = [
        ('odom',             '/odom_combined'),
        ('rgb/image',        '/camera/color/image_raw'),
        ('rgb/camera_info',  '/camera/color/camera_info'),
        ('depth/image',      '/camera/depth/image_raw'),
    ]

    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        DeclareLaunchArgument(
            'qos', default_value='1',
            description='QoS: 0=system default, 1=Reliable, 2=Best Effort'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true = localization mode (requires existing map)'),

        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz for visualization'),

        # ── Hardware bringup ─────────────────────────────────────────────────
        wheeltec_robot,
        wheeltec_camera,

        # ── RGB-D sync (approx_sync needed because color+depth timestamps differ) ──
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
                'approx_sync':           True,
                'approx_sync_max_interval': 0.05,
                'use_sim_time':          use_sim_time,
                'qos':                   qos,
            }],
            remappings=remappings,
        ),

        # ── SLAM mode ────────────────────────────────────────────────────────
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters],
            remappings=remappings,
            arguments=['-d'],   # delete db on start (fresh map each time)
        ),

        # ── Localization mode ─────────────────────────────────────────────────
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True',
            }],
            remappings=remappings,
        ),

        # ── Optional: RTAB-Map visualizer ────────────────────────────────────
        Node(
            condition=IfCondition(rviz),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
                'frame_id':     'camera_link',
                'use_sim_time': use_sim_time,
                'subscribe_rgbd': True,
                'qos':          qos,
            }],
            remappings=remappings,
        ),
    ])
