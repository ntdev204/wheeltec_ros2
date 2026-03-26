---
name: ros2-humble
description: ROS2 Humble development principles and best practices. Nodes, topics, services, actions, lifecycle management, package creation, launch systems, common packages, and simulation integration. Use when developing ROS2 applications, robotic systems, or autonomous systems with ROS2 Humble.
---

# ROS2 Humble

> Modern robotics middleware for distributed robotic systems.

## 1. Communication Patterns

| Pattern        | Characteristics                      | Use Cases                       |
| -------------- | ------------------------------------ | ------------------------------- |
| **Topics**     | Pub/Sub, many-to-many, continuous    | Sensor data streams             |
| **Services**   | Request/Response, 1-to-1,synchronous | Commands, queries               |
| **Actions**    | Goal-Feedback-Result, asynchronous   | Long-running tasks (navigation) |
| **Parameters** | Configuration, runtime updates       | Settings, tuning                |

### When to Use What

```
Continuous data stream?
├─ Yes → Topic (e.g., camera images, lidar)
└─ No → Service or Action

Need feedback during execution?
├─ Yes → Action (e.g., navigation, motion)
└─ No → Service (e.g., get config)

Task duration > 1 second?
├─ Yes → Action
└─ No → Service
```

---

## 2. Quality of Service (QoS)

| Profile            | Reliability | History               | Use Cases              |
| ------------------ | ----------- | --------------------- | ---------------------- |
| **Sensor Data**    | Best effort | Keep last (depth: 10) | High-frequency sensors |
| **Services**       | Reliable    | Keep last             | Request/response       |
| **Parameters**     | Reliable    | Keep all              | Configuration          |
| **System Default** | Reliable    | Keep last             | General purpose        |

**Common Issue:** QoS mismatch between publisher and subscriber causes no data flow.

---

## 3. Package Structure & Creation

### Standard Structure

```
my_package/
├── package.xml          # Metadata, dependencies
├── setup.py (Python) or CMakeLists.txt (C++)
├── my_package/          # Source code
├── launch/              # Launch files
├── config/              # YAML configs
├── msg/srv/action/      # Custom interfaces
└── test/                # Unit tests
```

### Create Package

```bash
# Python
ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy std_msgs

# C++
ros2 pkg create --build-type ament_cmake my_pkg --dependencies rclcpp std_msgs
```

**For detailed package structure, see [tf2-sim.md](references/tf2-sim.md)**

---

## 4. Node Development

### Basic Node Workflow

1. **Create node class** (inherit from `Node`)
2. **Initialize** publishers, subscribers, timers, services
3. **Implement callbacks**
4. **Spin** node in main()

### Python vs. C++

| Aspect          | Python (rclpy)      | C++ (rclcpp)         |
| --------------- | ------------------- | -------------------- |
| **Ease**        | Easier, prototyping | More complex         |
| **Performance** | Slower              | Faster, real-time    |
| **Use When**    | Rapid development   | Performance-critical |

**For code templates (pub/sub, service, action, lifecycle), see [node-development.md](references/node-development.md)**

---

## 5. Launch Files & Parameters

### Launch File Purpose

- Start multiple nodes
- Set parameters from YAML
- Remap topics
- Conditional launching
- Include other launch files

### Parameters Best Practices

- **Declare** parameters with defaults
- **Load** from YAML files
- **Dynamic updates** with callbacks

**For launch file examples and parameter patterns, see [launch-params.md](references/launch-params.md)**

---

## 6. Custom Interfaces

### Message (.msg)

```
std_msgs/Header header
int32 id
string name
float64[] data
```

### Service (.srv)

```
int32 request_id
---
bool success
string message
```

### Action (.action)

```
int32 target       # Goal
---
bool success       # Result
---
float32 progress   # Feedback
```

**Add to package.xml:**

```xml
<depend>rosidl_default_generators</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## 7. Common Packages

| Package           | Purpose          | Key Types                          |
| ----------------- | ---------------- | ---------------------------------- |
| **std_msgs**      | Basic types      | String, Int32, Float64, Header     |
| **sensor_msgs**   | Sensors          | Image, LaserScan, PointCloud2, Imu |
| **geometry_msgs** | Geometry         | Pose, Twist, Transform             |
| **nav_msgs**      | Navigation       | Odometry, Path                     |
| **tf2**           | Transforms       | Coordinate transformations         |
| **nav2**          | Navigation stack | Costmaps, planners, behaviors      |

**For TF2, Nav2, RViz, and Gazebo details, see [packages.md](references/packages.md)**

---

## 8. TF2 (Transforms)

### Purpose

Manage coordinate frame transformations (e.g., `world` → `robot` → `camera`).

### Usage

- **Broadcast** transforms (e.g., robot odometry)
- **Listen** to transforms (e.g., get camera pose in world frame)
- **Tree structure** (parent-child relationships)

### Common Commands

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transform
ros2 run tf2_ros tf2_echo world robot
```

**For TF2 code examples, see [packages.md](references/packages.md)**

---

## 9. Lifecycle Nodes

### States

```
Unconfigured → Inactive → Active → Inactive → Finalized
```

### When to Use

- **Managed startup/shutdown** (e.g., sensor initialization)
- **State management** (configure, activate, deactivate)
- **Production systems** (orderly lifecycle)

**For lifecycle implementation, see [node-development.md](references/node-development.md)**

---

## 10. Simulation (Gazebo & RViz)

### Gazebo

- **Physics simulation** (world, sensors, actuators)
- **Sensor plugins** (camera, lidar, IMU)
- **Robot spawning** from URDF/Xacro

### RViz2

- **Visualization tool** (not simulation)
- **Displays**: TF, LaserScan, PointCloud2, Camera, Robot Model

### URDF/Xacro

- **Robot description** format
- **Xacro**: XML macros for URDF

---

## 11. Best Practices

### Node Design

- **Single responsibility** per node
- **Composable nodes** (intra-process communication for performance)
- **Graceful shutdown** (destroy resources)

### Topic Naming Convention

```
/robot_name/sensor_type/data_type
Example: /robot1/camera/image_raw
```

### Performance

- Use **composable nodes** to minimize IPC overhead
- Profile with `ros2 topic hz` (frequency) and `ros2 topic bw` (bandwidth)
- **Zero-copy** transport when possible (intra-process)

---

## 12. Debugging Tools

| Tool             | Purpose                       | Example                                       |
| ---------------- | ----------------------------- | --------------------------------------------- |
| **ros2 topic**   | List, echo, publish, measure  | `ros2 topic echo /my_topic`                   |
| **ros2 node**    | List, info                    | `ros2 node list`                              |
| **ros2 param**   | Get, set, dump, load          | `ros2 param get /node param`                  |
| **ros2 service** | Call, list, type              | `ros2 service call /srv std_srvs/srv/Trigger` |
| **ros2 bag**     | Record, play, info            | `ros2 bag record /topic`                      |
| **rqt**          | GUI tools (graph, plot, etc.) | `rqt_graph`                                   |

**For full CLI command reference, see [packages.md](references/packages.md)**

---

## 13. Common Pitfalls

| Problem                  | Solution                                              |
| ------------------------ | ----------------------------------------------------- |
| **DDS discovery issues** | Check `ROS_DOMAIN_ID`, network, firewall              |
| **QoS incompatibility**  | Match QoS profiles between pub/sub                    |
| **Transform errors**     | Ensure all frames published, check TF tree            |
| **Parameter not found**  | Declare parameter before `get_parameter()`            |
| **Node name conflicts**  | Use unique names or namespaces                        |
| **Memory leaks**         | Proper shutdown, destroy nodes/subscriptions          |
| **Build errors**         | Check dependencies in `package.xml`, `rosdep install` |

---

## 14. Quick Start Workflow

1. **Create workspace**

    ```bash
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    ```

2. **Create package**

    ```bash
    ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy
    ```

3. **Write node** (see [node-development.md](references/node-development.md))

4. **Build**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_pkg
    source install/setup.bash
    ```

5. **Run**

    ```bash
    ros2 run my_pkg my_node
    ```

6. **Test communication**
    ```bash
    ros2 topic list
    ros2 topic echo /my_topic
    ```

---

## 15. Checklist for New ROS2 Project

- [ ] Define communication patterns (topics, services, actions)
- [ ] Design node architecture (single responsibility)
- [ ] Choose QoS profiles (sensor vs. reliable)
- [ ] Create package structure
- [ ] Implement nodes with proper lifecycle
- [ ] Write launch files
- [ ] Configure parameters (YAML files)
- [ ] Set up TF tree (coordinate frames)
- [ ] Test with RVzerviz2 visualization
- [ ] Simulate in Gazebo (if applicable)
- [ ] Write unit tests
- [ ] Profile performance (`ros2 topic hz/bw`)

---

> **Philosophy:** ROS2 is middleware, not a framework. Design loosely coupled nodes, use appropriate communication patterns, and leverage existing packages when possible.

## References

- [node-development.md](references/node-development.md) - Code templates (Python/C++, pub/sub, service, action, lifecycle)
- [launch-params.md](references/launch-params.md) - Launch files and parameters
- [packages.md](references/packages.md) - Common packages (TF2, Nav2, RViz, Gazebo, CLI)
- [tf2-sim.md](references/tf2-sim.md) - Package structure and best practices
