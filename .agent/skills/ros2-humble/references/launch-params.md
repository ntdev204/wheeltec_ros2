# ROS2 Launch Files & Parameters

## Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='my_package',
            executable='my_node',
            name='custom_name',
            parameters=[{'param_name': 'value'}],
            arguments=['--ros-args', '--log-level', 'INFO'],
            remappings=[('old_topic', 'new_topic')]
        )
    ])
```

## Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

include_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('other_package'),
        '/launch/other_launch.py'
    ])
)
```

## Parameters (YAML)

```yaml
my_node:
    ros__parameters:
        my_param: 42
        another_param: "hello"
        rate: 10.0
        nested:
            value: 3.14
            flag: true
```

## Loading Parameters in Node

```python
class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')

        # Declare with default
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('rate', 10.0)

        # Get value
        param_value = self.get_parameter('my_param').value
        rate = self.get_parameter('rate').value

        # Set callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed')
        return SetParametersResult(successful=True)
```

## Conditional Launch

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

Node(
    package='my_package',
    executable='my_node',
    condition=IfCondition(LaunchConfiguration('enable_node'))
)
```

## Group with Namespace

```python
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

GroupAction([
    PushRosNamespace('robot1'),
    Node(package='pkg', executable='node')
])
```
