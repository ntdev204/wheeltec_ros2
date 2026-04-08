from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ai_robot_msgs.msg import Detection, DetectionArray

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - runtime dependency on robot image stack
    CvBridge = None

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - optional dependency
    YOLO = None


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('model_path', 'yolov8n.engine')
        self.declare_parameter('conf_thres', 0.35)
        self.declare_parameter('device', '0')

        self._bridge: Optional[CvBridge] = None
        self._model = None
        self._sub = None
        self._pub = None
        self._image_topic = '/camera/image_raw'
        self._detection_topic = '/detections'
        self._conf_thres = 0.35
        self._device = '0'
        self._image_topic = self.get_parameter('image_topic').value
        self._detection_topic = self.get_parameter('detection_topic').value
        self._conf_thres = float(self.get_parameter('conf_thres').value)
        self._device = str(self.get_parameter('device').value)
        model_path = str(self.get_parameter('model_path').value)

        self._pub = self.create_publisher(DetectionArray, self._detection_topic, 10)
        self._sub = self.create_subscription(Image, self._image_topic, self._infer_cb, 10)

        if CvBridge is None:
            self.get_logger().error('cv_bridge is not installed. Detection will publish empty arrays.')
        else:
            self._bridge = CvBridge()

        if YOLO is None:
            self.get_logger().error('ultralytics is not installed. Detection will publish empty arrays.')
            self._model = None
            return

        try:
            self._model = YOLO(model_path)
            self.get_logger().info(f'Loaded YOLO model: {model_path}')
        except Exception as exc:  # pragma: no cover - depends on runtime files
            self._model = None
            self.get_logger().error(f'Failed to load YOLO model: {exc}')

    def _infer_cb(self, msg: Image) -> None:
        out = DetectionArray()
        out.header = msg.header

        if self._model is None or self._bridge is None:
            self._pub.publish(out)
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self._model.predict(
                source=frame,
                conf=self._conf_thres,
                device=self._device,
                verbose=False,
            )
        except Exception as exc:  # pragma: no cover - runtime behavior
            self.get_logger().warn(f'Inference failed, publishing empty detections: {exc}')
            self._pub.publish(out)
            return

        if not results:
            self._pub.publish(out)
            return

        names = getattr(self._model, 'names', {})
        for box in results[0].boxes:
            det = Detection()
            det.header = msg.header

            cls_idx = int(box.cls[0]) if box.cls is not None else -1
            if isinstance(names, dict):
                det.class_id = str(names.get(cls_idx, cls_idx))
            else:
                det.class_id = str(cls_idx)

            det.confidence = float(box.conf[0]) if box.conf is not None else 0.0

            xywh = box.xywh[0].tolist()
            det.center_x = float(xywh[0])
            det.center_y = float(xywh[1])
            det.size_x = float(xywh[2])
            det.size_y = float(xywh[3])

            out.detections.append(det)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
