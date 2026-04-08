import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String

from ai_robot_msgs.msg import TrackedHumanArray


class ContextNode(Node):
    def __init__(self) -> None:
        super().__init__('context')
        self.declare_parameter('tracked_topic', '/tracked_humans')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/ai_context/state')
        self.declare_parameter('speed_scale_topic', '/ai_context/speed_scale')

        self.declare_parameter('base_speed', 0.5)
        self.declare_parameter('density_threshold', 3.0)
        self.declare_parameter('stop_distance_m', 1.2)
        self.declare_parameter('clear_speed_scale', 1.0)
        self.declare_parameter('crowded_speed_scale', 0.4)

        self._sub = None
        self._cmd_pub = None
        self._state_pub = None
        self._scale_pub = None
        self._tracked_topic = self.get_parameter('tracked_topic').value
        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._state_topic = self.get_parameter('state_topic').value
        self._speed_scale_topic = self.get_parameter('speed_scale_topic').value

        self._base_speed = float(self.get_parameter('base_speed').value)
        self._density_threshold = float(self.get_parameter('density_threshold').value)
        self._stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self._clear_scale = float(self.get_parameter('clear_speed_scale').value)
        self._crowded_scale = float(self.get_parameter('crowded_speed_scale').value)

        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self._scale_pub = self.create_publisher(Float32, self._speed_scale_topic, 10)
        self._sub = self.create_subscription(TrackedHumanArray, self._tracked_topic, self._context_cb, 10)

    def _context_cb(self, msg: TrackedHumanArray) -> None:
        density = float(msg.crowd_density)
        closest = float(msg.closest_human_dist)

        if closest < self._stop_distance_m:
            scene_state = 'obstacle_near'
            speed_scale = 0.0
        elif density > self._density_threshold:
            scene_state = 'crowded'
            speed_scale = self._crowded_scale
        else:
            scene_state = 'clear'
            speed_scale = self._clear_scale

        cmd = Twist()
        cmd.linear.x = self._base_speed * speed_scale
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self._cmd_pub.publish(cmd)

        state_msg = String()
        state_msg.data = scene_state
        self._state_pub.publish(state_msg)

        scale_msg = Float32()
        scale_msg.data = float(speed_scale)
        self._scale_pub.publish(scale_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ContextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
