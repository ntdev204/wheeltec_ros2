from setuptools import setup
from glob import glob
import os

package_name = 'wheeltec_robot_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.inference',
        f'{package_name}.tracking',
        f'{package_name}.streaming',
        f'{package_name}.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wheeltec',
    maintainer_email='robot@wheeltec.net',
    description='AI detection and tracking package for Wheeltec robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = wheeltec_robot_detection.detection_node:main',
            'tracker_node = wheeltec_robot_detection.tracker_node:main',
            'video_stream_node = wheeltec_robot_detection.video_stream_node:main',
            'websocket_stream_node = wheeltec_robot_detection.websocket_stream_node:main',
            'dynamic_obstacle_publisher = wheeltec_robot_detection.dynamic_obstacle_publisher:main',
        ],
    },
)
