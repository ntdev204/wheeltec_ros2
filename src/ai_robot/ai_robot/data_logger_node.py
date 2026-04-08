import csv
import datetime
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from ai_robot_msgs.msg import DetectionArray, TrackedHumanArray


class DataLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('logger')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('tracked_topic', '/tracked_humans')
        self.declare_parameter('state_topic', '/ai_context/state')
        self.declare_parameter('speed_scale_topic', '/ai_context/speed_scale')
        self.declare_parameter('flush_service', '/ai_logger/flush')

        self.declare_parameter('log_dir', 'logs')
        self.declare_parameter('snapshot_period_sec', 1.0)
        self.declare_parameter('flush_period_sec', 300.0)

        self._sub_det = None
        self._sub_trk = None
        self._sub_state = None
        self._sub_scale = None
        self._timer = None
        self._srv = None

        self._buffer: List[Dict[str, object]] = []
        self._latest_num_persons = 0
        self._latest_density = 0.0
        self._latest_state = 'unknown'
        self._latest_scale = 1.0
        self._last_flush_time = None
        self._emergency_stops = 0
        self._detection_topic = self.get_parameter('detection_topic').value
        self._tracked_topic = self.get_parameter('tracked_topic').value
        self._state_topic = self.get_parameter('state_topic').value
        self._speed_scale_topic = self.get_parameter('speed_scale_topic').value
        self._flush_service = self.get_parameter('flush_service').value

        self._log_dir = Path(str(self.get_parameter('log_dir').value))
        self._snapshot_period = float(self.get_parameter('snapshot_period_sec').value)
        self._flush_period = float(self.get_parameter('flush_period_sec').value)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        self._last_flush_time = self.get_clock().now()

        self._srv = self.create_service(Trigger, self._flush_service, self._flush_service_cb)
        self._sub_det = self.create_subscription(DetectionArray, self._detection_topic, self._det_cb, 10)
        self._sub_trk = self.create_subscription(TrackedHumanArray, self._tracked_topic, self._trk_cb, 10)
        self._sub_state = self.create_subscription(String, self._state_topic, self._state_cb, 10)
        self._sub_scale = self.create_subscription(Float32, self._speed_scale_topic, self._scale_cb, 10)
        self._timer = self.create_timer(self._snapshot_period, self._snapshot_cb)

    def _det_cb(self, msg: DetectionArray) -> None:
        pass

    def _trk_cb(self, msg: TrackedHumanArray) -> None:
        self._latest_num_persons = len(msg.humans)
        self._latest_density = float(msg.crowd_density)

    def _state_cb(self, msg: String) -> None:
        self._latest_state = msg.data
        if msg.data == 'obstacle_near':
            self._emergency_stops += 1

    def _scale_cb(self, msg: Float32) -> None:
        self._latest_scale = float(msg.data)

    def _snapshot_cb(self) -> None:
        now = self.get_clock().now().to_msg()
        ts = float(now.sec) + float(now.nanosec) * 1e-9

        row = {
            'timestamp': datetime.datetime.fromtimestamp(ts).isoformat(),
            'num_persons': self._latest_num_persons,
            'crowd_density': self._latest_density,
            'scene_state': self._latest_state,
            'robot_speed_scale': self._latest_scale,
        }
        self._buffer.append(row)

        elapsed = (self.get_clock().now() - self._last_flush_time).nanoseconds / 1e9
        if elapsed >= self._flush_period:
            self._flush_all()

    def _flush_service_cb(self, request, response):
        self._flush_all()
        response.success = True
        response.message = f'Flushed logs to {self._log_dir.resolve()}'
        return response

    def _flush_all(self) -> None:
        if not self._buffer:
            self._last_flush_time = self.get_clock().now()
            return

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = self._log_dir / f'{stamp}_session.csv'
        report_path = self._log_dir / f'{stamp}_report.md'

        with csv_path.open('w', newline='') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=['timestamp', 'num_persons', 'crowd_density', 'scene_state', 'robot_speed_scale'],
            )
            writer.writeheader()
            writer.writerows(self._buffer)

        report = self._render_report()
        report_path.write_text(report)

        self.get_logger().info(f'Wrote CSV: {csv_path}')
        self.get_logger().info(f'Wrote report: {report_path}')

        self._buffer.clear()
        self._last_flush_time = self.get_clock().now()
        self._emergency_stops = 0

    def _render_report(self) -> str:
        total = len(self._buffer)
        if total == 0:
            return '# Session Report\n\nNo data collected.'

        avg_persons = sum(float(r['num_persons']) for r in self._buffer) / total
        avg_density = sum(float(r['crowd_density']) for r in self._buffer) / total

        states: Dict[str, int] = {}
        for row in self._buffer:
            s = str(row['scene_state'])
            states[s] = states.get(s, 0) + 1

        state_lines = ['| State | Count |', '|---|---:|']
        for name, count in sorted(states.items(), key=lambda it: it[0]):
            state_lines.append(f'| {name} | {count} |')

        return (
            f'# Session Report - {datetime.datetime.now().isoformat()}\\n\\n'
            '## Summary\\n'
            f'- Total snapshots: {total}\\n'
            f'- Avg persons/frame: {avg_persons:.2f}\\n'
            f'- Avg crowd density: {avg_density:.2f}\\n'
            f'- Emergency stops: {self._emergency_stops}\\n\\n'
            '## State Distribution\\n'
            + '\\n'.join(state_lines)
            + '\\n'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
