# Trình Tự Khởi Động Hệ Thống (Startup Sequence)

Tài liệu này hướng dẫn cách bật toàn bộ hệ thống Robot tự hành adaptive-context-aware theo đúng thứ tự để đảm bảo Raspberry Pi và laptop runtime kết nối, đồng bộ thành công.

---

## 1. Raspberry Pi (Controller & ROS2 Core)
**Vai trò:** Giao tiếp phần cứng, đọc Lidar, phân luồng vận tốc (Twist Mux) và giao tiếp với Web SCADA.
**IP Mặc định:** `25.12.4.101`

Mở các terminal SSH vào Pi và chạy lần lượt:

### Terminal 1: Bật Base & Cảm biến (Lidar, Motor)
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py
```
*(Nếu bạn có dùng file `prod_bringup.launch.py` thì dùng thay thế)*

### Terminal 2: Bật Trình phân luồng ưu tiên (Twist Mux & AI Bridge)
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch wheeltec_twist_mux twist_mux.launch.py adaptive_host:=<LAPTOP_IP>
```
*(Lưu ý: File launch này vẫn gọi node `context_aware_bridge` để giữ tương thích launch cũ, nhưng node đã nói protocol adaptive-context-aware: ZMQ sensor ingest `5555`, ZMQ result `5556`, TCP NAV_CMD `9091`, TCP heartbeat `9093`)*

### Terminal 3: Bật Cầu nối SCADA (Tùy chọn nếu dùng Web)
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 run wheeltec_scada_bridge bridge_node
```

---

## 2. Laptop (AI Core + Web SCADA)
**Vai trò:** Chạy adaptive-context-aware runtime, nhận sensor protobuf từ Raspberry Pi, publish perception result, heartbeat, và lệnh NAV_CMD.
**IP:** dùng IP laptop mà Raspberry Pi truy cập được, ví dụ IP Tailscale hoặc IP LAN.

⚠️ **Lưu ý:** Khởi động Raspberry Pi bridge trước giúp laptop runtime có sẵn các cổng TCP nhận NAV_CMD/heartbeat và kênh sensor ingest kết nối ổn định hơn.

Mở terminal trên laptop:

### Terminal 1: Khởi chạy adaptive-context-aware runtime
```bash
cd D:/utc/deep/adaptive-context-aware
docker compose -f docker/docker-compose.yml up -d control-api
```
*(Quá trình này bật FastAPI control plane, ZMQ/protobuf data plane, heartbeat và các pipeline adaptive reasoning.)*

Bạn có thể xem log bằng lệnh:
```bash
docker compose -f docker/docker-compose.yml logs -f control-api
```
**Vai trò Web:** Trạm điều khiển mặt đất (Ground Control Station), hiển thị bản đồ, telemetry, camera AI và nút dừng khẩn cấp.

Mở terminal trên máy tính Windows/Mac của bạn (tại thư mục mã nguồn project):

### Khởi chạy Web bằng Docker
```bash
cd D:/utc/deep/rai_website
docker compose up -d
```
*(Lệnh này sẽ tự động khởi tạo và chạy cả Backend (FastAPI) và Frontend (Next.js) trong container)*

### Giao diện Web:
Mở trình duyệt (Chrome/Edge) và truy cập: **`http://localhost:3000`**

---

## 🛑 Cơ chế an toàn (Arbitration)
Khi cả hệ thống hoạt động, mạch phân luồng `twist_mux` trên Raspberry Pi sẽ tự động quyết định xem robot sẽ nghe lệnh của ai theo thứ tự ưu tiên:
1. **Mức 10 (Cao nhất):** Bàn phím máy tính hoặc Joypad trên Web (`/cmd_vel_keyboard`). Khi bạn bấm phím, AI sẽ lập tức bị ghi đè.
2. **Mức 6:** Điều hướng tự động Nav2 (`/cmd_vel_nav`).
3. **Mức 5:** Lệnh của laptop adaptive-context-aware (`/cmd_vel_context`). Khi Nav2 không phát lệnh, adaptive runtime có thể điều khiển robot.

---

## 4. Tự động chạy khi Raspberry Pi khởi động (systemd)
Để tự động chạy lệnh:
```bash
ros2 launch turn_on_wheeltec_robot prod_bringup.launch.py
```

Trong repo đã có sẵn:
- `scripts/start_prod_bringup.sh`
- `systemd/wheeltec-prod-bringup.service`

Chạy các lệnh sau trên Raspberry Pi:
```bash
cd ~/wheeltec_ros2
chmod +x scripts/start_prod_bringup.sh
sudo cp systemd/wheeltec-prod-bringup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wheeltec-prod-bringup.service
sudo systemctl start wheeltec-prod-bringup.service
```

Kiểm tra trạng thái:
```bash
systemctl status wheeltec-prod-bringup.service
journalctl -u wheeltec-prod-bringup.service -f
```
