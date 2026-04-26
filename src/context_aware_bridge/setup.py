from setuptools import find_packages, setup

package_name = 'context_aware_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/context_aware_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pyzmq'],
    zip_safe=True,
    maintainer='thientn204',
    maintainer_email='thientn204@gmail.com',
    description=(
        'ZMQ bridge: Jetson AI NavigationCommand (struct binary) → ROS2 /cmd_vel_context; '
        '/odom → RobotState (Protobuf/struct) → Jetson'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'context_aware_bridge = context_aware_bridge.main:main',
        ],
    },
)
