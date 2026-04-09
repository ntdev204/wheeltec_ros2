#!/usr/bin/env python3
"""YOLOv8s TensorRT detection node — decoupled inference thread for 35+ FPS.

Key fixes vs previous version:
  1. CUDA context: make_context() auto-pushes on constructor thread.
     The inference thread must POP from main thread first, then push on
     inference thread. Fixed via ctx.detach() + push() on inference thread.
  2. MultiThreadedExecutor: guarantees callbacks are non-blocking.
  3. frame.copy() inside lock eliminated — use double-buffer swap instead.
  4. preview publish uses bgr8 raw (not JPEG) — cv2_to_imgmsg is near-zero cost.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import time
import threading
from pathlib import Path
import yaml

from wheeltec_robot_msg.msg import Detection2D, Detection2DArray
from wheeltec_robot_detection.inference.tensorrt_detector import TensorRTDetector


class DetectionNode(Node):
    """ROS2 node — camera callback stores frame; inference thread runs TRT at full speed."""

    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        if config_file and Path(config_file).exists():
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.get_logger().warn('No config file — using defaults')
            self.config = self._default_config()

        detection_config = self.config['detection']
        topics_config    = self.config['topics']

        self.bridge = CvBridge()

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, topics_config['camera_input'],
            self.image_callback, camera_qos,
        )
        self.detections_pub = self.create_publisher(
            Detection2DArray, topics_config['detections_output'], 10,
        )
        self.fps_pub = self.create_publisher(
            Float32, topics_config['fps_output'], 10,
        )
        self.latency_pub = self.create_publisher(
            Float32, topics_config['latency_output'], 10,
        )
        self.preview_pub = self.create_publisher(
            Image, '/ai/preview', 1,
        )

        # Double-buffer: camera writes to _write_buf; inference reads from _read_buf
        # Lock only protects the swap, not the actual data access
        self._swap_lock    = threading.Lock()
        self._write_buf    = None   # latest incoming frame
        self._write_header = None
        self._read_buf     = None   # frame owned by inference thread
        self._read_header  = None
        self._has_new      = False  # flag: new frame available since last inference

        # FPS tracking
        self._inf_count        = 0
        self._inf_window_start = time.monotonic()

        # Load model and start inference thread
        self._running    = True
        self._detection_config = detection_config
        self._inf_thread = threading.Thread(
            target=self._inference_loop, name='trt_inference', daemon=True,
        )
        self._inf_thread.start()

        self.get_logger().info('Detection node initialized (decoupled inference thread)')

    def image_callback(self, msg: Image):
        """Cache latest frame — never blocks, never copies more than needed."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        with self._swap_lock:
            # Overwrite write buf — inference will grab it on next iter
            self._write_buf    = frame          # no .copy() — inference owns read_buf separately
            self._write_header = msg.header
            self._has_new      = True

    def _inference_loop(self):
        """Inference thread — owns CUDA context exclusively."""
        # ── CRITICAL: Load detector here, on the inference thread.
        # make_context() pushes ctx on THIS thread. No inter-thread ctx ownership issue.
        detection_config = self._detection_config
        self.get_logger().info(f"Loading model: {detection_config['model_path']}")
        try:
            self.detector = TensorRTDetector(
                model_path=detection_config['model_path'],
                input_size=tuple(detection_config['input_size']),
                confidence_threshold=detection_config['confidence_threshold'],
                nms_threshold=detection_config['nms_threshold'],
                class_names=detection_config['class_names'],
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load TensorRT engine: {e}')
            return
        self.get_logger().info('TensorRT detector loaded — inference starting')

        while self._running:
            # ── Grab new frame via swap (O(1), no copy)
            with self._swap_lock:
                if not self._has_new or self._write_buf is None:
                    frame  = None
                    header = None
                else:
                    # Swap: inference thread takes ownership of write_buf
                    frame  = self._write_buf
                    header = self._write_header
                    # Give write_buf a fresh slot (inference now owns old write_buf)
                    self._write_buf = None
                    self._has_new   = False

            if frame is None:
                time.sleep(0.001)   # 1ms yield when no new frame
                continue

            # ── Inference
            t0 = time.monotonic()
            try:
                detections = self.detector.detect(frame)
            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')
                continue
            latency_ms = (time.monotonic() - t0) * 1000.0

            # ── Build Detection2DArray
            det_array        = Detection2DArray()
            det_array.header = header
            h, w             = frame.shape[:2]

            for det in detections:
                d            = Detection2D()
                d.header     = header
                d.class_name = det['class_name']
                d.class_id   = det['class_id']
                d.confidence = det['confidence']
                x1, y1, x2, y2 = det['bbox']
                d.x_min = x1; d.y_min = y1
                d.x_max = x2; d.y_max = y2
                d.x_center = (x1 + x2) / 2.0 / w
                d.y_center = (y1 + y2) / 2.0 / h
                d.width    = (x2 - x1) / w
                d.height   = (y2 - y1) / h
                det_array.detections.append(d)

            self.detections_pub.publish(det_array)

            # ── Preview: 320x240 downsample (keeps DDS traffic low)
            preview = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)
            preview_msg = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
            preview_msg.header = header
            self.preview_pub.publish(preview_msg)

            # ── Latency
            lat_msg      = Float32()
            lat_msg.data = latency_ms
            self.latency_pub.publish(lat_msg)

            # ── FPS counter (1-second window)
            self._inf_count += 1
            now     = time.monotonic()
            elapsed = now - self._inf_window_start
            if elapsed >= 1.0:
                fps                    = self._inf_count / elapsed
                self._inf_count        = 0
                self._inf_window_start = now

                fps_msg      = Float32()
                fps_msg.data = fps
                self.fps_pub.publish(fps_msg)

                self.get_logger().info(
                    f'FPS: {fps:.1f} | Latency: {latency_ms:.1f}ms | '
                    f'Detections: {len(detections)}'
                )

        # Cleanup
        try:
            self.detector.cuda_ctx.pop()
        except Exception:
            pass

    def _default_config(self):
        return {
            'detection': {
                'model_path': '/home/robot/wheeltec_ros2/src/wheeltec_robot_detection/models/yolov8s.engine',
                'input_size': [640, 640],
                'confidence_threshold': 0.5,
                'nms_threshold': 0.45,
                'class_names': ['person', 'bicycle', 'car'],
            },
            'topics': {
                'camera_input':      '/camera/color/image_raw',
                'detections_output': '/detections',
                'fps_output':        '/ai/fps',
                'latency_output':    '/ai/latency',
            },
        }


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    # MultiThreadedExecutor: camera callback never blocks inference thread publication
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node._inf_thread.join(timeout=3.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
