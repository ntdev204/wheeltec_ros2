"""Adaptive-context-aware bridge for Wheeltec ROS2.

This keeps the historical package/executable name so existing launch files and
twist_mux wiring continue to work, but it speaks the new adaptive data plane:

* ROS2 /scan, /imu/data_raw and Pi status -> adaptive runtime ZMQ PUSH protobuf
* Adaptive runtime ZMQ PUB perception results -> ROS2 diagnostics JSON
* Adaptive runtime TCP NAV_CMD packets (9091) -> /cmd_vel_context
* Optional heartbeat watchdog server (9093) -> fail-safe stop
"""

from __future__ import annotations

import json
import math
import socket
import struct
import time
from dataclasses import dataclass
from threading import Event, Lock, Thread

import rclpy
import zmq
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String

from context_aware_bridge.obstacle_guard import ObstacleGuard

MAGIC = b"\xCA\xFE"
HEADER_FORMAT = "!2s B I Q I"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_FORMAT = "!H"
CRC_SIZE = struct.calcsize(CRC_FORMAT)

MSG_NAV_CMD = 0x02
MSG_HEARTBEAT = 0x03
MSG_ACK = 0x06

NAV_CMD_FORMAT = "!f f f I H H"
HEARTBEAT_FORMAT = "!B f B H"
ACK_FORMAT = "!B I B H"

MAX_LINEAR_VEL = 1.0
MAX_ANGULAR_VEL = 2.0
LIDAR_MAX_DISTANCE = 9.9
LIDAR_SCAN_BINS = 360
_SENSOR_CLASSES = None
_PERCEPTION_ENVELOPE_CLASS = None


@dataclass(frozen=True, slots=True)
class Packet:
    msg_type: int
    seq: int
    timestamp_us: int
    payload: bytes


class ContextAwareBridgeNode(Node):
    def __init__(self):
        super().__init__("context_aware_bridge")

        self.declare_parameter("adaptive_host", "")
        self.declare_parameter("jetson_ip", "127.0.0.1")
        self.declare_parameter("raspi_ip", "25.12.4.101")
        self.declare_parameter("sensor_ingest_port", 5555)
        self.declare_parameter("result_publish_port", 5556)
        self.declare_parameter("nav_cmd_port", 9091)
        self.declare_parameter("heartbeat_port", 9093)
        self.declare_parameter("source_id", "wheeltec-raspi")
        self.declare_parameter("publish_lidar_hz", 10.0)
        self.declare_parameter("publish_imu_hz", 30.0)
        self.declare_parameter("publish_status_hz", 1.0)
        self.declare_parameter("watchdog_timeout_sec", 1.5)
        self.declare_parameter("enable_heartbeat_watchdog", True)

        self.declare_parameter("stop_radius", 0.50)
        self.declare_parameter("slow_radius", 0.70)
        self.declare_parameter("influence_radius", 1.00)
        self.declare_parameter("narrow_stop_radius", 0.30)
        self.declare_parameter("narrow_slow_radius", 0.50)
        self.declare_parameter("narrow_mode", False)
        self.declare_parameter("lidar_offset_m", 0.10)
        self.declare_parameter("scan_stale_timeout", 0.75)

        adaptive_host = str(self.get_parameter("adaptive_host").value)
        legacy_jetson_ip = str(self.get_parameter("jetson_ip").value)
        self._adaptive_host = adaptive_host or legacy_jetson_ip or "127.0.0.1"
        self._source_id = str(self.get_parameter("source_id").value)
        self._sensor_ingest_port = int(self.get_parameter("sensor_ingest_port").value)
        self._result_publish_port = int(self.get_parameter("result_publish_port").value)
        self._nav_cmd_port = int(self.get_parameter("nav_cmd_port").value)
        self._heartbeat_port = int(self.get_parameter("heartbeat_port").value)
        self._watchdog_timeout_sec = float(self.get_parameter("watchdog_timeout_sec").value)
        self._enable_heartbeat_watchdog = bool(self.get_parameter("enable_heartbeat_watchdog").value)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_context", 10)
        self.result_pub = self.create_publisher(String, "/adaptive_context/perception_result", 10)

        self.create_subscription(LaserScan, "/scan", self._scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data_raw", self._imu_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Float32, "/PowerVoltage", self._voltage_cb, 10)

        self._ctx = zmq.Context()
        self._sensor_push = self._ctx.socket(zmq.PUSH)
        self._sensor_push.setsockopt(zmq.SNDHWM, 1)
        self._sensor_push.setsockopt(zmq.LINGER, 0)
        self._sensor_push.connect(f"tcp://{self._adaptive_host}:{self._sensor_ingest_port}")

        self._result_sub = self._ctx.socket(zmq.SUB)
        self._result_sub.setsockopt(zmq.RCVHWM, 2)
        self._result_sub.setsockopt(zmq.LINGER, 0)
        self._result_sub.setsockopt(zmq.RCVTIMEO, 200)
        self._result_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self._result_sub.connect(f"tcp://{self._adaptive_host}:{self._result_publish_port}")

        self._lock = Lock()
        self._stop_event = Event()
        self._lidar_scan360 = [LIDAR_MAX_DISTANCE] * LIDAR_SCAN_BINS
        self._lidar_points: list[tuple[float, float]] = []
        self._lidar_valid = False
        self._last_scan_time = 0.0
        self._imu_payload: dict | None = None
        self._voltage = 0.0
        self._seq = 0
        self._last_cmd_time = time.monotonic()
        self._last_heartbeat_time = time.monotonic()
        self._watchdog_stop_sent = False

        self._obstacle_guard = ObstacleGuard(
            stop_radius=self.get_parameter("stop_radius").value,
            slow_radius=self.get_parameter("slow_radius").value,
            narrow_stop_radius=self.get_parameter("narrow_stop_radius").value,
            narrow_slow_radius=self.get_parameter("narrow_slow_radius").value,
            influence_radius=self.get_parameter("influence_radius").value,
            scan_stale_timeout=self.get_parameter("scan_stale_timeout").value,
            lidar_offset_m=self.get_parameter("lidar_offset_m").value,
        )
        self._obstacle_guard.set_narrow_mode(self.get_parameter("narrow_mode").value)

        self._threads = [
            Thread(target=self._result_loop, daemon=True, name="adaptive-result-sub"),
            Thread(target=self._nav_cmd_server, daemon=True, name="adaptive-nav-server"),
        ]
        if self._enable_heartbeat_watchdog:
            self._threads.append(Thread(target=self._heartbeat_server, daemon=True, name="adaptive-heartbeat-server"))
        for thread in self._threads:
            thread.start()

        self.create_timer(1.0 / max(1.0, float(self.get_parameter("publish_lidar_hz").value)), self._publish_lidar)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("publish_imu_hz").value)), self._publish_imu)
        self.create_timer(1.0 / max(0.2, float(self.get_parameter("publish_status_hz").value)), self._publish_status)
        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            "[adaptive-context-aware bridge] started\n"
            f"  PUSH sensors  tcp://{self._adaptive_host}:{self._sensor_ingest_port}\n"
            f"  SUB results   tcp://{self._adaptive_host}:{self._result_publish_port}\n"
            f"  TCP nav_cmd   0.0.0.0:{self._nav_cmd_port}\n"
            f"  TCP heartbeat 0.0.0.0:{self._heartbeat_port}"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        points = []
        scan360 = [LIDAR_MAX_DISTANCE] * LIDAR_SCAN_BINS
        angle = msg.angle_min
        for dist in msg.ranges:
            if math.isfinite(dist) and msg.range_min < dist < msg.range_max:
                clipped = min(float(dist), LIDAR_MAX_DISTANCE)
                points.append((float(angle), clipped))
                deg = int(round(math.degrees(angle))) % LIDAR_SCAN_BINS
                scan360[deg] = min(scan360[deg], clipped)
            angle += msg.angle_increment

        with self._lock:
            self._lidar_points = points
            self._lidar_scan360 = scan360
            self._lidar_valid = bool(points)
            self._last_scan_time = time.monotonic()

    def _imu_cb(self, msg: Imu) -> None:
        with self._lock:
            self._imu_payload = {
                "accel": (
                    float(msg.linear_acceleration.x),
                    float(msg.linear_acceleration.y),
                    float(msg.linear_acceleration.z),
                ),
                "quat": (
                    float(msg.orientation.x),
                    float(msg.orientation.y),
                    float(msg.orientation.z),
                    float(msg.orientation.w),
                ),
            }

    def _voltage_cb(self, msg: Float32) -> None:
        with self._lock:
            self._voltage = float(msg.data)

    def _publish_lidar(self) -> None:
        with self._lock:
            points = list(self._lidar_points)
        if not points:
            return
        payload = self._encode_sensor_envelope(
            payload_field="lidar_scan",
            payload={
                "angle_rad": [angle for angle, _ in points],
                "range_m": [distance for _, distance in points],
            },
        )
        self._send_sensor_payload(payload)

    def _publish_imu(self) -> None:
        with self._lock:
            imu = dict(self._imu_payload) if self._imu_payload else None
        if not imu:
            return
        accel = imu["accel"]
        quat = imu["quat"]
        payload = self._encode_sensor_envelope(
            payload_field="imu_sample",
            payload={
                "accel_x_mps2": accel[0],
                "accel_y_mps2": accel[1],
                "accel_z_mps2": accel[2],
                "quat_x": quat[0],
                "quat_y": quat[1],
                "quat_z": quat[2],
                "quat_w": quat[3],
            },
        )
        self._send_sensor_payload(payload)

    def _publish_status(self) -> None:
        with self._lock:
            voltage = self._voltage
        payload = self._encode_sensor_envelope(
            payload_field="pi_status",
            payload={
                "state": "normal" if voltage >= 21.0 or voltage == 0.0 else "low_battery",
                "cpu_temp_c": 0.0,
                "cpu_load_pct": 0.0,
            },
        )
        self._send_sensor_payload(payload)

    def _send_sensor_payload(self, payload: bytes) -> None:
        try:
            self._sensor_push.send(payload, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as exc:
            self.get_logger().warning(f"sensor publish failed: {exc}")

    def _result_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                raw = self._result_sub.recv()
                result = _decode_perception_result(raw)
                msg = String()
                msg.data = json.dumps(result, separators=(",", ":"))
                self.result_pub.publish(msg)
            except zmq.Again:
                continue
            except zmq.ContextTerminated:
                break
            except Exception as exc:
                self.get_logger().warning(f"adaptive result decode failed: {exc}")

    def _nav_cmd_server(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", self._nav_cmd_port))
            server.listen(8)
            server.settimeout(0.2)
            while rclpy.ok() and not self._stop_event.is_set():
                try:
                    conn, _ = server.accept()
                except socket.timeout:
                    continue
                Thread(target=self._handle_nav_client, args=(conn,), daemon=True).start()

    def _handle_nav_client(self, conn: socket.socket) -> None:
        with conn:
            stream = conn.makefile("rb")
            while rclpy.ok() and not self._stop_event.is_set():
                try:
                    packet = _read_packet(stream)
                except (ConnectionError, OSError, ValueError):
                    break
                if packet.msg_type != MSG_NAV_CMD:
                    continue
                try:
                    vx, vy, omega, _, _, _ = struct.unpack(NAV_CMD_FORMAT, packet.payload)
                except struct.error:
                    continue
                self._publish_nav_twist(vx, vy, omega)

    def _publish_nav_twist(self, vx: float, vy: float, omega: float) -> None:
        twist = Twist()
        twist.linear.x = _clamp(float(vx), -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        twist.linear.y = _clamp(float(vy), -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        twist.angular.z = _clamp(float(omega), -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

        with self._lock:
            scan360 = list(self._lidar_scan360) if self._lidar_valid else None
            scan_age_s = time.monotonic() - self._last_scan_time

        twist = self._obstacle_guard.guard(twist, scan360, scan_age_s)
        self.cmd_vel_pub.publish(twist)
        self._last_cmd_time = time.monotonic()
        self._watchdog_stop_sent = False

    def _heartbeat_server(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", self._heartbeat_port))
            server.listen(8)
            server.settimeout(0.2)
            while rclpy.ok() and not self._stop_event.is_set():
                try:
                    conn, _ = server.accept()
                except socket.timeout:
                    continue
                Thread(target=self._handle_heartbeat_client, args=(conn,), daemon=True).start()

    def _handle_heartbeat_client(self, conn: socket.socket) -> None:
        with conn:
            stream = conn.makefile("rb")
            while rclpy.ok() and not self._stop_event.is_set():
                try:
                    packet = _read_packet(stream)
                except (ConnectionError, OSError, ValueError):
                    break
                if packet.msg_type == MSG_HEARTBEAT:
                    self._last_heartbeat_time = time.monotonic()
                    ack = _encode_packet(MSG_ACK, self._next_seq(), struct.pack(ACK_FORMAT, MSG_HEARTBEAT, packet.seq, 0, 0))
                    try:
                        conn.sendall(ack)
                    except OSError:
                        break

    def _watchdog_tick(self) -> None:
        now = time.monotonic()
        command_stale = now - self._last_cmd_time > self._watchdog_timeout_sec
        heartbeat_stale = self._enable_heartbeat_watchdog and now - self._last_heartbeat_time > 2.0
        if (command_stale or heartbeat_stale) and not self._watchdog_stop_sent:
            self.cmd_vel_pub.publish(Twist())
            self._watchdog_stop_sent = True

    def _next_seq(self) -> int:
        value = self._seq
        self._seq += 1
        return value

    def _encode_sensor_envelope(self, *, payload_field: str, payload: dict) -> bytes:
        sequence = self._next_seq()
        timestamp_us = int(time.time() * 1_000_000)
        if payload_field == "lidar_scan":
            payload_bytes = _encode_lidar_scan(payload)
            payload_number = 10
        elif payload_field == "imu_sample":
            payload_bytes = _encode_imu_sample(payload)
            payload_number = 11
        elif payload_field == "pi_status":
            payload_bytes = _encode_pi_status(payload)
            payload_number = 12
        else:
            raise ValueError(f"unsupported sensor payload field: {payload_field}")

        return b"".join(
            (
                _encode_string_field(1, self._source_id),
                _encode_varint_field(2, sequence),
                _encode_varint_field(3, timestamp_us),
                _encode_message_field(payload_number, payload_bytes),
            )
        )

    def destroy_node(self) -> None:
        self._stop_event.set()
        try:
            self._sensor_push.close(linger=0)
            self._result_sub.close(linger=0)
            self._ctx.term()
        finally:
            super().destroy_node()


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def _crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def _encode_packet(msg_type: int, seq: int, payload: bytes) -> bytes:
    header = struct.pack(HEADER_FORMAT, MAGIC, msg_type, seq, int(time.time() * 1_000_000), len(payload))
    crc = _crc16_ccitt(header + payload)
    return header + payload + struct.pack(CRC_FORMAT, crc)


def _read_exact(stream, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            raise ConnectionError("stream closed")
        chunks.extend(chunk)
    return bytes(chunks)


def _read_packet(stream) -> Packet:
    header = _read_exact(stream, HEADER_SIZE)
    magic, msg_type, seq, timestamp_us, payload_len = struct.unpack(HEADER_FORMAT, header)
    if magic != MAGIC:
        raise ValueError("invalid magic")
    payload_and_crc = _read_exact(stream, payload_len + CRC_SIZE)
    raw = header + payload_and_crc
    payload = payload_and_crc[:-CRC_SIZE]
    (received_crc,) = struct.unpack(CRC_FORMAT, payload_and_crc[-CRC_SIZE:])
    if received_crc != _crc16_ccitt(raw[:-CRC_SIZE]):
        raise ValueError("crc mismatch")
    return Packet(msg_type=msg_type, seq=seq, timestamp_us=timestamp_us, payload=payload)


def _decode_perception_result(raw: bytes) -> dict:
    envelope = _decode_result_envelope(raw)
    return {
        "source_id": envelope["source_id"],
        "sequence": envelope["sequence"],
        "timestamp_us": envelope["timestamp_us"],
        "entities": envelope["entities"],
        "metrics": envelope["metrics"],
    }


def _encode_lidar_scan(payload: dict) -> bytes:
    return b"".join(
        (
            _encode_packed_float_field(1, payload["angle_rad"]),
            _encode_packed_float_field(2, payload["range_m"]),
        )
    )


def _encode_imu_sample(payload: dict) -> bytes:
    fields = (
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
        "quat_x",
        "quat_y",
        "quat_z",
        "quat_w",
    )
    return b"".join(_encode_float_field(index, payload[name]) for index, name in enumerate(fields, start=1))


def _encode_pi_status(payload: dict) -> bytes:
    return b"".join(
        (
            _encode_string_field(1, payload["state"]),
            _encode_float_field(2, payload["cpu_temp_c"]),
            _encode_float_field(3, payload["cpu_load_pct"]),
        )
    )


def _encode_key(field_number: int, wire_type: int) -> bytes:
    return _encode_varint((field_number << 3) | wire_type)


def _encode_varint(value: int) -> bytes:
    value = int(value)
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def _encode_varint_field(field_number: int, value: int) -> bytes:
    return _encode_key(field_number, 0) + _encode_varint(value)


def _encode_bytes_field(field_number: int, value: bytes) -> bytes:
    return _encode_key(field_number, 2) + _encode_varint(len(value)) + value


def _encode_string_field(field_number: int, value: str) -> bytes:
    return _encode_bytes_field(field_number, str(value).encode("utf-8"))


def _encode_message_field(field_number: int, value: bytes) -> bytes:
    return _encode_bytes_field(field_number, value)


def _encode_float_field(field_number: int, value: float) -> bytes:
    return _encode_key(field_number, 5) + struct.pack("<f", float(value))


def _encode_packed_float_field(field_number: int, values) -> bytes:
    body = b"".join(struct.pack("<f", float(value)) for value in values)
    return _encode_bytes_field(field_number, body)


def _decode_result_envelope(raw: bytes) -> dict:
    envelope = {
        "source_id": "",
        "sequence": 0,
        "timestamp_us": 0,
        "entities": [],
        "metrics": {
            "total_latency_ms": 0.0,
            "camera_latency_ms": 0.0,
            "detector_latency_ms": 0.0,
            "fusion_latency_ms": 0.0,
            "fps": 0.0,
        },
    }
    for field_number, wire_type, value in _iter_proto_fields(raw):
        if field_number == 1 and wire_type == 2:
            envelope["source_id"] = value.decode("utf-8", errors="replace")
        elif field_number == 2 and wire_type == 0:
            envelope["sequence"] = int(value)
        elif field_number == 3 and wire_type == 0:
            envelope["timestamp_us"] = int(value)
        elif field_number == 10 and wire_type == 2:
            envelope["entities"].append(_decode_tracked_entity(value))
        elif field_number == 11 and wire_type == 2:
            envelope["metrics"] = _decode_runtime_metrics(value)
    return envelope


def _decode_tracked_entity(raw: bytes) -> dict:
    entity = {
        "track_id": 0,
        "bbox_xywh": [],
        "position_xyz_m": [],
        "velocity_xyz_mps": [],
        "heading_rad": 0.0,
        "confidence": 0.0,
        "nearest_obstacle_distance_m": None,
    }
    for field_number, wire_type, value in _iter_proto_fields(raw):
        if field_number == 1 and wire_type == 0:
            entity["track_id"] = int(value)
        elif field_number == 2:
            entity["bbox_xywh"].extend(_decode_float_values(wire_type, value))
        elif field_number == 3:
            entity["position_xyz_m"].extend(_decode_float_values(wire_type, value))
        elif field_number == 4:
            entity["velocity_xyz_mps"].extend(_decode_float_values(wire_type, value))
        elif field_number == 5 and wire_type == 5:
            entity["heading_rad"] = value
        elif field_number == 6 and wire_type == 5:
            entity["confidence"] = value
        elif field_number == 7 and wire_type == 5:
            entity["nearest_obstacle_distance_m"] = value
    return entity


def _decode_runtime_metrics(raw: bytes) -> dict:
    metrics = {
        "total_latency_ms": 0.0,
        "camera_latency_ms": 0.0,
        "detector_latency_ms": 0.0,
        "fusion_latency_ms": 0.0,
        "fps": 0.0,
    }
    names = {
        1: "total_latency_ms",
        2: "camera_latency_ms",
        3: "detector_latency_ms",
        4: "fusion_latency_ms",
        5: "fps",
    }
    for field_number, wire_type, value in _iter_proto_fields(raw):
        if wire_type == 5 and field_number in names:
            metrics[names[field_number]] = value
    return metrics


def _iter_proto_fields(raw: bytes):
    offset = 0
    while offset < len(raw):
        key, offset = _read_varint_from(raw, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type == 0:
            value, offset = _read_varint_from(raw, offset)
        elif wire_type == 2:
            size, offset = _read_varint_from(raw, offset)
            value = raw[offset : offset + size]
            offset += size
        elif wire_type == 5:
            value = struct.unpack_from("<f", raw, offset)[0]
            offset += 4
        else:
            raise ValueError(f"unsupported protobuf wire type: {wire_type}")
        yield field_number, wire_type, value


def _read_varint_from(raw: bytes, offset: int) -> tuple[int, int]:
    shift = 0
    value = 0
    while offset < len(raw):
        byte = raw[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
        shift += 7
    raise ValueError("truncated protobuf varint")


def _decode_float_values(wire_type: int, value) -> list[float]:
    if wire_type == 5:
        return [float(value)]
    if wire_type != 2:
        return []
    if len(value) % 4 != 0:
        raise ValueError("packed float field length must be a multiple of 4")
    return [struct.unpack_from("<f", value, index)[0] for index in range(0, len(value), 4)]
