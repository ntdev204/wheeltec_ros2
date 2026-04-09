#!/usr/bin/env python3
"""
ROS2 Pipeline Speed Probe — attach to a running pipeline and report Hz of all key topics.
Run: python3 speed_probe.py
Press Ctrl+C to stop.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from wheeltec_robot_msg.msg import Detection2DArray
import time, collections

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class SpeedProbe(Node):
    def __init__(self):
        super().__init__('speed_probe')
        self.counters = collections.defaultdict(int)
        self.last_ts  = collections.defaultdict(lambda: time.monotonic())
        self.hz_vals  = collections.defaultdict(list)

        # Subscribe to every key topic
        self.create_subscription(Image,           '/camera/color/image_raw', lambda m: self._tick('camera_raw'),       BEST_EFFORT_QOS)
        self.create_subscription(Image,           '/ai/preview',             lambda m: self._tick('ai_preview'),       BEST_EFFORT_QOS)
        self.create_subscription(Detection2DArray,'/detections',             lambda m: self._tick('detections'),       10)
        self.create_subscription(Float32,         '/ai/fps',                 lambda m: self._tick_fps(m),              10)
        self.create_subscription(Float32,         '/ai/latency',             lambda m: self._tick_lat(m),              10)

        self.latest_fps     = None
        self.latest_latency = None

        # Print report every 2 seconds
        self.create_timer(2.0, self._report)
        self.get_logger().info('SpeedProbe attached — waiting for topics...')

    def _tick(self, key):
        now = time.monotonic()
        dt  = now - self.last_ts[key]
        self.last_ts[key] = now
        if dt > 0 and dt < 5.0:
            self.hz_vals[key].append(1.0 / dt)
            if len(self.hz_vals[key]) > 30:
                self.hz_vals[key].pop(0)

    def _tick_fps(self, msg):
        self.latest_fps = msg.data
        self._tick('ai_fps')

    def _tick_lat(self, msg):
        self.latest_latency = msg.data
        self._tick('ai_latency')

    def _hz(self, key):
        vals = self.hz_vals[key]
        if not vals:
            return "  --.- Hz  (no msgs)"
        avg = sum(vals) / len(vals)
        return f"{avg:6.1f} Hz  (n={len(vals)})"

    def _report(self):
        sep = "=" * 55
        fps_str = f"{self.latest_fps:.1f}" if self.latest_fps else "N/A"
        lat_str = f"{self.latest_latency:.1f}ms" if self.latest_latency else "N/A"
        print(f"\n{sep}")
        print(f"  /camera/color/image_raw  {self._hz('camera_raw')}")
        print(f"  /ai/preview              {self._hz('ai_preview')}")
        print(f"  /detections              {self._hz('detections')}")
        print(f"  TRT FPS (from /ai/fps):  {fps_str}")
        print(f"  TRT Latency:             {lat_str}")
        print(sep)

def main():
    rclpy.init()
    node = SpeedProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
