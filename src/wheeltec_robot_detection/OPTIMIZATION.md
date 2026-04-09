# Tối ưu Pipeline để đạt 25+ FPS

## Thay đổi chính

### 1. Loại bỏ ZMQ - Chuyển sang WebSocket
- **Trước**: ZMQ pub/sub với encode/decode overhead
- **Sau**: WebSocket trực tiếp với FastAPI, giảm latency ~5-10ms

### 2. Tích hợp Web Server vào Jetson
- Web server chạy cùng ROS2 node trên Jetson
- Không cần máy tính riêng để host web
- Giảm network hops, tăng throughput

### 3. Tối ưu encoding
- Giảm JPEG quality từ 75 → 70 (tăng ~3-5 FPS)
- Async broadcast đến multiple clients
- Zero-copy khi có thể

## Cài đặt

```bash
cd /home/robot/wheeltec_ros2/src/wheeltec_robot_detection
pip3 install -r requirements.txt
```

## Build

```bash
cd /home/robot/wheeltec_ros2
colcon build --packages-select wheeltec_robot_detection
source install/setup.bash
```

## Chạy pipeline mới

```bash
ros2 launch wheeltec_robot_detection optimized_pipeline.launch.py
```

## Truy cập web interface

Mở browser và truy cập:
```
http://<JETSON_IP>:8000
```

Ví dụ: `http://192.168.1.100:8000`

## Kiểm tra FPS

```bash
ros2 topic echo /ai/fps
```

## So sánh hiệu năng

| Metric | Pipeline cũ (ZMQ) | Pipeline mới (WebSocket) |
|--------|-------------------|--------------------------|
| Latency | ~50-70ms | ~30-40ms |
| FPS | 18-22 | 25-30+ |
| Network overhead | Cao (ZMQ + Web server riêng) | Thấp (WebSocket trực tiếp) |
| Setup complexity | 2 máy (Jetson + Web server) | 1 máy (Jetson) |

## Tối ưu thêm (nếu cần)

### Giảm resolution preview
Trong `detection_node.py:180`, thay đổi:
```python
preview = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)
```
Thành:
```python
preview = cv2.resize(frame, (240, 180), interpolation=cv2.INTER_LINEAR)
```

### Giảm JPEG quality thêm
Trong `config/streaming_params.yaml`:
```yaml
quality: 65  # từ 70 xuống 65
```

### Giảm target FPS nếu không cần 30fps
```yaml
target_fps: 25  # từ 30 xuống 25
```

## Troubleshooting

### Port 8000 đã được sử dụng
Thay đổi port trong `config/streaming_params.yaml`:
```yaml
port: 8080  # hoặc port khác
```

### FPS vẫn thấp
1. Kiểm tra GPU usage: `tegrastats`
2. Kiểm tra TensorRT engine đã optimize chưa
3. Giảm resolution hoặc quality như hướng dẫn trên
