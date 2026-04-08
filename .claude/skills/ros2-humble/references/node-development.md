# ROS2 Node Development - Code Templates

## Python Node Template

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(String, 'input', self.callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS2'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## C++ Node Template

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "input", 10, std::bind(&MyNode::callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MyNode::timer_callback, this));
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS2";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Service Server/Client

### Python Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

### Python Service Client

```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## Action Server/Client

### Python Action Server

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci', self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        # Send feedback
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        return result
```

## Lifecycle Node

```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def on_configure(self, state: State):
        self.get_logger().info('Configuring')
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating')
        # Start publishing/subscribing
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating')
        # Pause
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up')
        # Destroy resources
        return TransitionCallbackReturn.SUCCESS
```

## Custom Messages/Services/Actions

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
string command
---
bool success
string message
```

### Action (.action)

```
# Goal
int32 target
---
# Result
bool success
---
# Feedback
float32 progress
```

## QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Sensor data QoS
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Reliable QoS
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```
