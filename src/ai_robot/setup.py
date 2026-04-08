from setuptools import find_packages, setup

package_name = 'ai_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ai_system.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='thientn204',
    maintainer_email='thientn204@gmail.com',
    description='Lifecycle AI stack for Wheeltec robot on Jetson',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node = ai_robot.detection_node:main',
            'detection_overlay_node = ai_robot.detection_overlay_node:main',
            'tracking_node = ai_robot.tracking_node:main',
            'context_node = ai_robot.context_node:main',
            'data_logger_node = ai_robot.data_logger_node:main',
            'pi4_serial_bridge = ai_robot.pi4_serial_bridge:main',
        ],
    },
)
