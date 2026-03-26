# ROS2 Package Structure & Best Practices

## Standard Package Structure

```
my_package/
├── package.xml          # Package metadata
├── setup.py             # Python package setup (ament_python)
├── setup.cfg            # Package configuration
├── CMakeLists.txt       # C++ build (ament_cmake)
├── my_package/          # Python source
│   ├── __init__.py
│   └── my_node.py
├── launch/              # Launch files
│   └── my_launch.py
├── config/              # Configuration files
│   └── params.yaml
├── msg/                 # Custom messages
│   └── CustomMsg.msg
├── srv/                 # Custom services
│   └── CustomSrv.srv
├── action/              # Custom actions
│   └── CustomAction.action
├── urdf/                # Robot descriptions
│   └── robot.urdf.xacro
├── rviz/                # RViz configs
│   └── config.rviz
└── test/                # Tests
    └── test_my_node.py
```

## Package Creation

```bash
# Python package
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs

# C++ package
ros2 pkg create --build-type ament_cmake my_package \
  --dependencies rclcpp std_msgs

# Mixed
ros2 pkg create --build-type ament_cmake my_package \
  --dependencies rclcpp rclpy
```

## package.xml (Example)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My ROS2 package</description>
  <maintainer email="user@email.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Best Practices

### Node Design

- **Single responsibility** per node
- Use **composition** for multi-node processes
- Implement **lifecycle** for managed nodes
- Handle **shutdown** gracefully

### Topic Naming

```
/robot_name/sensor_type/data_type
Example: /robot1/camera/image_raw
```

### QoS Selection

- **Sensor data**: Best effort, small depth (10)
- **Commands**: Reliable, depth 10
- **State**: Reliable, transient local

### Performance

- Use **composable nodes** (intra-process communication)
- Minimize message copies
- Profile with `ros2 topic hz/bw`
- Use **zero-copy** when possible

## Debugging

| Issue                     | Solution                                           |
| ------------------------- | -------------------------------------------------- |
| **Nodes not discovering** | Check `ROS_DOMAIN_ID`, DDS settings                |
| **QoS incompatibility**   | Match QoS profiles between pub/sub                 |
| **Transform errors**      | Ensure all frames published, check with `tf2_echo` |
| **Parameter not found**   | Declare parameters before use                      |
| **Build errors**          | Check dependencies in `package.xml`                |

## Testing

```python
import unittest
from my_package.my_node import MyNode
import rclpy

class TestMyNode(unittest.TestCase):
    def test_something(self):
        rclpy.init()
        node = MyNode()
        # Test logic
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
```

## Build & Run

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash

# Run node
ros2 run my_package my_node

# Launch file
ros2 launch my_package my_launch.py
```
