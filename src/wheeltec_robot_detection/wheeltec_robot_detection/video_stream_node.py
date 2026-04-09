#!/usr/bin/env python3
"""Video streaming node — ZMQ encode/send in background thread to avoid blocking ROS spin."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
import yaml
import cv2
import zmq
import numpy as np
import threading
import queue

from wheeltec_robot_msg.msg import Detection2DArray, TrackedHumanArray


class VideoStreamNode(Node):
    """Streams annotated video via ZMQ; encode/send runs in a background thread."""

    def __init__(self):
        super().__init__('video_stream_node')

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        if config_file and Path(config_file).exists():
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.get_logger().warn('No config file provided, using defaults')
            self.config = self._default_config()

        streaming_config = self.config['streaming']
        topics_config    = streaming_config['topics']

        # ZMQ sockets
        self.zmq_context   = zmq.Context()
        self.detection_pub = self.zmq_context.socket(zmq.PUB)
        self.detection_pub.bind(f"tcp://0.0.0.0:{streaming_config['ai_detection_port']}")
        self.tracking_pub  = self.zmq_context.socket(zmq.PUB)
        self.tracking_pub.bind(f"tcp://0.0.0.0:{streaming_config['human_tracking_port']}")

        # Encoding settings
        self.format        = streaming_config['format']
        self.quality       = streaming_config['quality']
        self.encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        self.bbox_thickness  = streaming_config['bbox_thickness']
        self.font_scale      = streaming_config['font_scale']
        self.font_thickness  = streaming_config['font_thickness']
        self.colors          = streaming_config['colors']

        self.bridge = CvBridge()

        # Cached ROS data
        self.latest_image      = None
        self.latest_detections = None
        self.latest_humans     = None

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, topics_config['camera_input'],
            self.image_callback, camera_qos,
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, topics_config['detections_input'],
            self.detection_callback, 10,
        )
        self.human_sub = self.create_subscription(
            TrackedHumanArray, topics_config['tracked_humans_input'],
            self.human_callback, 10,
        )

        # Background encode/send queue (maxsize=2 → drop when encoder is behind)
        self._stream_queue  = queue.Queue(maxsize=2)
        self._encode_thread = threading.Thread(
            target=self._encode_loop, name='zmq_encode', daemon=True,
        )
        self._encode_thread.start()

        target_fps = streaming_config['target_fps']
        self.create_timer(1.0 / target_fps, self.stream_callback)

        self.get_logger().info(
            f'Video streaming initialized (ports: {streaming_config["ai_detection_port"]}, '
            f'{streaming_config["human_tracking_port"]})'
        )

    # ── ROS callbacks — return immediately, never block ───────────────────────

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def detection_callback(self, msg: Detection2DArray):
        self.latest_detections = msg

    def human_callback(self, msg: TrackedHumanArray):
        self.latest_humans = msg

    def stream_callback(self):
        """Timer — snapshot and hand off to encode thread (non-blocking)."""
        if self.latest_image is None:
            return
        snapshot = (
            self.latest_image.copy(),
            self.latest_detections,
            self.latest_humans,
        )
        try:
            self._stream_queue.put_nowait(snapshot)
        except queue.Full:
            pass  # drop frame — encoder busy

    # ── Encode thread — JPEG + ZMQ here, NOT on ROS spin thread ──────────────

    def _encode_loop(self):
        while True:
            try:
                image, detections, humans = self._stream_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            try:
                if detections is not None:
                    det_frame = self._annotate_detections(image.copy(), detections)
                    ok, buf   = cv2.imencode(f'.{self.format}', det_frame, self.encode_params)
                    if ok:
                        self.detection_pub.send(b'DETECTION:' + buf.tobytes(), zmq.NOBLOCK)

                if humans is not None:
                    trk_frame = self._annotate_humans(image.copy(), humans)
                    ok, buf   = cv2.imencode(f'.{self.format}', trk_frame, self.encode_params)
                    if ok:
                        self.tracking_pub.send(b'TRACKING:' + buf.tobytes(), zmq.NOBLOCK)

            except Exception as e:
                self.get_logger().error(f'Streaming error: {e}')

    # ── Annotation helpers ────────────────────────────────────────────────────

    def _annotate_detections(self, image: np.ndarray,
                              detections: Detection2DArray) -> np.ndarray:
        for det in detections.detections:
            if det.class_name == 'person':
                color = tuple(self.colors['person'])
            elif det.class_name in ['car', 'truck', 'bus', 'motorcycle', 'bicycle']:
                color = tuple(self.colors['vehicle'])
            else:
                color = tuple(self.colors['default'])

            cv2.rectangle(image, (det.x_min, det.y_min), (det.x_max, det.y_max),
                          color, self.bbox_thickness)
            label = f'{det.class_name} {det.confidence:.2f}'
            (lw, lh), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, self.font_thickness,
            )
            cv2.rectangle(image, (det.x_min, det.y_min - lh - 5),
                          (det.x_min + lw, det.y_min), color, -1)
            cv2.putText(image, label, (det.x_min, det.y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_scale,
                        (0, 0, 0), self.font_thickness)
        return image

    def _annotate_humans(self, image: np.ndarray,
                          humans: TrackedHumanArray) -> np.ndarray:
        color = tuple(self.colors['person'])
        for h in humans.humans:
            cv2.rectangle(image, (h.x_min, h.y_min), (h.x_max, h.y_max),
                          color, self.bbox_thickness)
            label = f'ID:{h.track_id} {h.confidence:.2f}'
            (lw, lh), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, self.font_thickness,
            )
            cv2.rectangle(image, (h.x_min, h.y_min - lh - 5),
                          (h.x_min + lw, h.y_min), color, -1)
            cv2.putText(image, label, (h.x_min, h.y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_scale,
                        (0, 0, 0), self.font_thickness)

            if abs(h.velocity_x) > 1 or abs(h.velocity_y) > 1:
                cx = (h.x_min + h.x_max) // 2
                cy = (h.y_min + h.y_max) // 2
                cv2.arrowedLine(
                    image, (cx, cy),
                    (int(cx + h.velocity_x * 5), int(cy + h.velocity_y * 5)),
                    color, 2, tipLength=0.3,
                )
        return image

    def _default_config(self):
        return {
            'streaming': {
                'ai_detection_port':   5558,
                'human_tracking_port': 5559,
                'format':          'jpg',
                'quality':         75,
                'target_fps':      30,
                'bbox_thickness':  2,
                'font_scale':      0.5,
                'font_thickness':  1,
                'colors': {
                    'person':  [0, 255, 0],
                    'vehicle': [255, 0, 0],
                    'default': [0, 255, 255],
                },
                'topics': {
                    'camera_input':         '/ai/preview',
                    'detections_input':     '/detections',
                    'tracked_humans_input': '/tracked_humans',
                },
            }
        }


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
