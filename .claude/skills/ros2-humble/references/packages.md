# ROS2 Common Packages

## Core Packages

| Package           | Purpose                | Key Messages/Services                          |
| ----------------- | ---------------------- | ---------------------------------------------- |
| **std_msgs**      | Standard message types | String, Int32, Float64, Bool, Header           |
| **sensor_msgs**   | Sensor data            | Image, LaserScan, PointCloud2, Imu, JointState |
| **geometry_msgs** | Geometry               | Pose, Twist, Transform, Point, Quaternion      |
| **nav_msgs**      | Navigation             | Odometry, Path, OccupancyGrid                  |
| **tf2**           | Transform library      | TF tree, coordinate transformations            |

## TF2 (Transforms)

### Broadcasting Transforms

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast)

    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'
        t.transform.translation.x = 1.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
```

### Listening to Transforms

```python
from tf2_ros import Buffer, TransformListener

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'robot', rclpy.time.Time())
            return trans
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
```

## Navigation (Nav2)

### Key Concepts

- **Costmaps**: Obstacle representation
- **Planners**: Global (A\*) and local (DWB)
- **Behavior Tree**: Mission planning
- **Recovery**: Stuck recovery behaviors

### Basic Nav2 Stack

```bash
ros2 launch nav2_bringup navigation_launch.py
```

## RViz2 Visualization

### Common Displays

- **TF**: Coordinate frames
- **Robot Model**: URDF visualization
- **LaserScan**: 2D lidar
- **PointCloud2**: 3D lidar
- **Camera**: Image
- **Path**: Planned paths
- **Map**: Occupancy grid

### Launch RViz with Config

```bash
ros2 run rviz2 rviz2 -d config.rviz
```

## Gazebo Simulation

### Launch Gazebo with ROS2

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

### Spawn Robot

```python
from gazebo_msgs.srv import SpawnEntity

client = self.create_client(SpawnEntity, '/spawn_entity')
request = SpawnEntity.Request()
request.name = 'my_robot'
request.xml = urdf_content
client.call_async(request)
```

## Common CLI Commands

```bash
# Node info
ros2 node list
ros2 node info /my_node

# Topic operations
ros2 topic list
ros2 topic echo /my_topic
ros2 topic pub /my_topic std_msgs/msg/String "data: 'hello'"
ros2 topic hz /my_topic
ros2 topic bw /my_topic

# Service operations
ros2 service list
ros2 service call /my_service std_srvs/srv/Trigger

# Parameter operations
ros2 param list
ros2 param get /my_node my_param
ros2 param set /my_node my_param 42

# Bag recording
ros2 bag record /topic1 /topic2
ros2 bag play my_bag

# TF operations
ros2 run tf2_ros tf2_echo world robot
```
