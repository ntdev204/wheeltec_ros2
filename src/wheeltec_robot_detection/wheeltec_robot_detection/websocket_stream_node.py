#!/usr/bin/env python3
"""WebSocket streaming node — direct stream from detection, no ZMQ overhead."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
import yaml
import cv2
import numpy as np
import asyncio
import threading
from typing import Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from wheeltec_robot_msg.msg import Detection2DArray, TrackedHumanArray


class WebSocketStreamNode(Node):
    """Streams video via WebSocket — zero-copy JPEG encode, async broadcast."""

    def __init__(self):
        super().__init__('websocket_stream_node')

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        if config_file and Path(config_file).exists():
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.get_logger().warn('No config file, using defaults')
            self.config = self._default_config()

        streaming_config = self.config['streaming']
        topics_config = streaming_config['topics']

        self.quality = streaming_config['quality']
        self.encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        self.bbox_thickness = streaming_config['bbox_thickness']
        self.font_scale = streaming_config['font_scale']
        self.font_thickness = streaming_config['font_thickness']
        self.colors = streaming_config['colors']
        self.port = streaming_config.get('port', 8000)

        self.bridge = CvBridge()

        # Cached data
        self.latest_image = None
        self.latest_detections = None
        self.latest_humans = None
        self._lock = threading.Lock()

        # WebSocket clients
        self.detection_clients: Set[WebSocket] = set()
        self.tracking_clients: Set[WebSocket] = set()
        self._event_loop = None  # Will be set by uvicorn thread

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

        # Start FastAPI server in background thread
        self.app = self._create_app()
        self.server_thread = threading.Thread(
            target=self._run_server, daemon=True
        )
        self.server_thread.start()

        # Stream timer
        target_fps = streaming_config['target_fps']
        self.create_timer(1.0 / target_fps, self.stream_callback)

        self.get_logger().info(f'WebSocket streaming on port {self.port}')

    def _create_app(self) -> FastAPI:
        app = FastAPI()

        @app.get("/", response_class=HTMLResponse)
        async def index():
            return """
<!DOCTYPE html>
<html>
<head>
    <title>Wheeltec Robot Vision</title>
    <style>
        body { margin: 0; padding: 20px; background: #1a1a1a; color: #fff; font-family: Arial; }
        .container { max-width: 1400px; margin: 0 auto; }
        h1 { text-align: center; }
        .streams { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .stream { background: #2a2a2a; padding: 15px; border-radius: 8px; }
        .stream h2 { margin-top: 0; color: #4CAF50; }
        img { width: 100%; height: auto; border-radius: 4px; }
        .status { padding: 10px; background: #333; border-radius: 4px; margin-top: 10px; }
        .connected { color: #4CAF50; }
        .disconnected { color: #f44336; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Wheeltec Robot Vision System</h1>
        <div class="streams">
            <div class="stream">
                <h2>AI Detection</h2>
                <img id="detection" src="" alt="Detection stream">
                <div class="status">Status: <span id="det-status" class="disconnected">Disconnected</span></div>
            </div>
            <div class="stream">
                <h2>Human Tracking</h2>
                <img id="tracking" src="" alt="Tracking stream">
                <div class="status">Status: <span id="trk-status" class="disconnected">Disconnected</span></div>
            </div>
        </div>
    </div>
    <script>
        const detImg = document.getElementById('detection');
        const trkImg = document.getElementById('tracking');
        const detStatus = document.getElementById('det-status');
        const trkStatus = document.getElementById('trk-status');

        function connectWebSocket(url, imgElement, statusElement) {
            const ws = new WebSocket(url);
            ws.binaryType = 'arraybuffer';

            ws.onopen = () => {
                statusElement.textContent = 'Connected';
                statusElement.className = 'connected';
            };

            ws.onmessage = (event) => {
                const blob = new Blob([event.data], { type: 'image/jpeg' });
                const url = URL.createObjectURL(blob);
                if (imgElement.src) URL.revokeObjectURL(imgElement.src);
                imgElement.src = url;
            };

            ws.onerror = () => {
                statusElement.textContent = 'Error';
                statusElement.className = 'disconnected';
            };

            ws.onclose = () => {
                statusElement.textContent = 'Disconnected';
                statusElement.className = 'disconnected';
                setTimeout(() => connectWebSocket(url, imgElement, statusElement), 3000);
            };
        }

        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsHost = window.location.host;
        connectWebSocket(`${wsProtocol}//${wsHost}/ws/detection`, detImg, detStatus);
        connectWebSocket(`${wsProtocol}//${wsHost}/ws/tracking`, trkImg, trkStatus);
    </script>
</body>
</html>
"""

        @app.websocket("/ws/detection")
        async def detection_websocket(websocket: WebSocket):
            await websocket.accept()
            self.detection_clients.add(websocket)
            try:
                while True:
                    await websocket.receive_text()
            except WebSocketDisconnect:
                self.detection_clients.discard(websocket)

        @app.websocket("/ws/tracking")
        async def tracking_websocket(websocket: WebSocket):
            await websocket.accept()
            self.tracking_clients.add(websocket)
            try:
                while True:
                    await websocket.receive_text()
            except WebSocketDisconnect:
                self.tracking_clients.discard(websocket)

        return app

    def _run_server(self):
        # Capture event loop for thread-safe coroutine scheduling
        self._event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._event_loop)
        uvicorn.run(self.app, host='0.0.0.0', port=self.port, log_level='warning')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self.latest_image = frame
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def detection_callback(self, msg: Detection2DArray):
        with self._lock:
            self.latest_detections = msg

    def human_callback(self, msg: TrackedHumanArray):
        with self._lock:
            self.latest_humans = msg

    def stream_callback(self):
        with self._lock:
            if self.latest_image is None:
                return
            image = self.latest_image.copy()
            detections = self.latest_detections
            humans = self.latest_humans

        # Encode and broadcast detection stream
        if self.detection_clients and detections is not None and self._event_loop:
            det_frame = self._annotate_detections(image.copy(), detections)
            ok, buf = cv2.imencode('.jpg', det_frame, self.encode_params)
            if ok:
                asyncio.run_coroutine_threadsafe(
                    self._broadcast(self.detection_clients, buf.tobytes()),
                    self._event_loop
                )

        # Encode and broadcast tracking stream
        if self.tracking_clients and humans is not None and self._event_loop:
            trk_frame = self._annotate_humans(image.copy(), humans)
            ok, buf = cv2.imencode('.jpg', trk_frame, self.encode_params)
            if ok:
                asyncio.run_coroutine_threadsafe(
                    self._broadcast(self.tracking_clients, buf.tobytes()),
                    self._event_loop
                )

    async def _broadcast(self, clients: Set[WebSocket], data: bytes):
        dead_clients = set()
        for client in clients:
            try:
                await client.send_bytes(data)
            except:
                dead_clients.add(client)
        clients -= dead_clients

    def _annotate_detections(self, image: np.ndarray, detections: Detection2DArray) -> np.ndarray:
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
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                          self.font_scale, self.font_thickness)
            cv2.rectangle(image, (det.x_min, det.y_min - lh - 5),
                          (det.x_min + lw, det.y_min), color, -1)
            cv2.putText(image, label, (det.x_min, det.y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_scale,
                        (0, 0, 0), self.font_thickness)
        return image

    def _annotate_humans(self, image: np.ndarray, humans: TrackedHumanArray) -> np.ndarray:
        color = tuple(self.colors['person'])
        for h in humans.humans:
            cv2.rectangle(image, (h.x_min, h.y_min), (h.x_max, h.y_max),
                          color, self.bbox_thickness)
            label = f'ID:{h.track_id} {h.confidence:.2f}'
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                          self.font_scale, self.font_thickness)
            cv2.rectangle(image, (h.x_min, h.y_min - lh - 5),
                          (h.x_min + lw, h.y_min), color, -1)
            cv2.putText(image, label, (h.x_min, h.y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_scale,
                        (0, 0, 0), self.font_thickness)

            if abs(h.velocity_x) > 1 or abs(h.velocity_y) > 1:
                cx = (h.x_min + h.x_max) // 2
                cy = (h.y_min + h.y_max) // 2
                cv2.arrowedLine(image, (cx, cy),
                                (int(cx + h.velocity_x * 5), int(cy + h.velocity_y * 5)),
                                color, 2, tipLength=0.3)
        return image

    def _default_config(self):
        return {
            'streaming': {
                'port': 8000,
                'quality': 75,
                'target_fps': 30,
                'bbox_thickness': 2,
                'font_scale': 0.5,
                'font_thickness': 1,
                'colors': {
                    'person': [0, 255, 0],
                    'vehicle': [255, 0, 0],
                    'default': [0, 255, 255],
                },
                'topics': {
                    'camera_input': '/ai/preview',
                    'detections_input': '/detections',
                    'tracked_humans_input': '/tracked_humans',
                },
            }
        }


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
