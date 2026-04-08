import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial
except ImportError:  # pragma: no cover - optional dependency
    serial = None


class Pi4SerialBridge(Node):
    def __init__(self) -> None:
        super().__init__('pi4_serial_bridge')
        self.declare_parameter('cmd_topic', '/cmd_vel_ai')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed_mps', 1.0)

        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._serial_port = self.get_parameter('serial_port').value
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._max_speed = max(0.01, float(self.get_parameter('max_speed_mps').value))

        self._ser = None
        if serial is None:
            self.get_logger().error('pyserial is not installed. Bridge will not transmit.')
        else:
            try:
                self._ser = serial.Serial(self._serial_port, self._baudrate, timeout=0.1)
                self.get_logger().info(f'Serial connected: {self._serial_port} @ {self._baudrate}')
            except Exception as exc:
                self.get_logger().error(f'Failed to open serial {self._serial_port}: {exc}')

        self.create_subscription(Twist, self._cmd_topic, self._cmd_cb, 10)

    def _cmd_cb(self, msg: Twist) -> None:
        if self._ser is None:
            return
        speed_norm = max(0.0, min(1.0, msg.linear.x / self._max_speed))
        speed_byte = int(speed_norm * 100)
        frame = bytes([0xAA, speed_byte, 0xFF])
        try:
            self._ser.write(frame)
        except Exception as exc:
            self.get_logger().error(f'Serial write failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = Pi4SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
