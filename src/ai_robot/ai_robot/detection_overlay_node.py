from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ai_robot_msgs.msg import Detection, DetectionArray

try:
    import cv2
except ImportError:  # pragma: no cover - runtime dependency
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - runtime dependency
    CvBridge = None


class DetectionOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('detection_overlay')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('overlay_image_topic', '/camera/detection_overlay')
        self.declare_parameter('label_font_scale', 0.5)
        self.declare_parameter('line_thickness', 2)

        self._image_topic = self.get_parameter('image_topic').value
        self._detection_topic = self.get_parameter('detection_topic').value
        self._overlay_image_topic = self.get_parameter('overlay_image_topic').value
        self._label_font_scale = float(self.get_parameter('label_font_scale').value)
        self._line_thickness = int(self.get_parameter('line_thickness').value)

        self._bridge = CvBridge() if CvBridge is not None else None
        self._latest_detections: List[Detection] = []

        if cv2 is None or self._bridge is None:
            self.get_logger().error('OpenCV or cv_bridge missing. Overlay node disabled.')
            return

        self._pub = self.create_publisher(Image, self._overlay_image_topic, 10)
        self.create_subscription(DetectionArray, self._detection_topic, self._detection_cb, 10)
        self.create_subscription(Image, self._image_topic, self._image_cb, 10)
        self.get_logger().info(f'Overlay topic: {self._overlay_image_topic}')

    def _detection_cb(self, msg: DetectionArray) -> None:
        self._latest_detections = list(msg.detections)

    def _image_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        for det in self._latest_detections:
            x1 = int(det.center_x - det.size_x / 2.0)
            y1 = int(det.center_y - det.size_y / 2.0)
            x2 = int(det.center_x + det.size_x / 2.0)
            y2 = int(det.center_y + det.size_y / 2.0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), self._line_thickness)
            label = f'{det.class_id} {det.confidence:.2f}'
            cv2.putText(
                frame,
                label,
                (x1, max(15, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                self._label_font_scale,
                (0, 255, 0),
                max(1, self._line_thickness - 1),
                cv2.LINE_AA,
            )

        out = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
