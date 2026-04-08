import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from ai_robot_msgs.msg import DetectionArray, TrackedHuman, TrackedHumanArray


@dataclass
class TrackState:
    track_id: int
    cx: float
    cy: float
    vx: float
    vy: float
    stamp_sec: float


class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('tracker')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('tracked_topic', '/tracked_humans')
        self.declare_parameter('person_only', True)
        self.declare_parameter('match_distance_px', 90.0)
        self.declare_parameter('track_timeout_sec', 1.0)
        self.declare_parameter('fov_area_m2', 1.0)
        self.declare_parameter('focal_length_px', 900.0)
        self.declare_parameter('person_height_m', 1.7)

        self._sub = None
        self._pub = None
        self._tracks: Dict[int, TrackState] = {}
        self._next_track_id = 1
        self._detection_topic = self.get_parameter('detection_topic').value
        self._tracked_topic = self.get_parameter('tracked_topic').value
        self._person_only = bool(self.get_parameter('person_only').value)
        self._match_distance_px = float(self.get_parameter('match_distance_px').value)
        self._track_timeout_sec = float(self.get_parameter('track_timeout_sec').value)
        self._fov_area_m2 = max(0.01, float(self.get_parameter('fov_area_m2').value))
        self._focal_length_px = float(self.get_parameter('focal_length_px').value)
        self._person_height_m = float(self.get_parameter('person_height_m').value)

        self._pub = self.create_publisher(TrackedHumanArray, self._tracked_topic, 10)
        self._sub = self.create_subscription(DetectionArray, self._detection_topic, self._track_cb, 10)

    def _track_cb(self, msg: DetectionArray) -> None:
        now_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        detections: List[Tuple[float, float, object]] = []

        for det in msg.detections:
            if self._person_only and str(det.class_id).lower() != 'person':
                continue
            detections.append((det.center_x, det.center_y, det))

        assigned_track_ids = set()
        matched = []

        for cx, cy, det in detections:
            best_id = None
            best_dist = self._match_distance_px
            for tid, track in self._tracks.items():
                if tid in assigned_track_ids:
                    continue
                dist = math.hypot(cx - track.cx, cy - track.cy)
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is None:
                best_id = self._next_track_id
                self._next_track_id += 1
                vx = 0.0
                vy = 0.0
            else:
                prev = self._tracks[best_id]
                dt = max(1e-3, now_sec - prev.stamp_sec)
                vx = (cx - prev.cx) / dt
                vy = (cy - prev.cy) / dt

            self._tracks[best_id] = TrackState(best_id, cx, cy, vx, vy, now_sec)
            assigned_track_ids.add(best_id)
            matched.append((best_id, det, vx, vy))

        stale_ids = [
            tid for tid, track in self._tracks.items()
            if (now_sec - track.stamp_sec) > self._track_timeout_sec and tid not in assigned_track_ids
        ]
        for tid in stale_ids:
            del self._tracks[tid]

        out = TrackedHumanArray()
        out.header = msg.header

        closest = float('inf')
        for tid, det, vx, vy in matched:
            human = TrackedHuman()
            human.header = msg.header
            human.track_id = tid
            human.center_x = float(det.center_x)
            human.center_y = float(det.center_y)
            human.size_x = float(det.size_x)
            human.size_y = float(det.size_y)
            human.velocity.x = float(vx)
            human.velocity.y = float(vy)
            human.velocity.z = 0.0
            human.confidence = float(det.confidence)

            pix_h = max(1.0, float(det.size_y))
            human.distance_m = float((self._focal_length_px * self._person_height_m) / pix_h)
            closest = min(closest, human.distance_m)
            out.humans.append(human)

        out.crowd_density = float(len(out.humans) / self._fov_area_m2)
        out.closest_human_dist = float(closest if out.humans else 999.0)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
