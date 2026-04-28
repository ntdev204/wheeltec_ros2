# Trình Tự Khởi Động Hệ Thống (Startup Sequence)

Tài liệu này hướng dẫn cách bật toàn bộ hệ thống Robot tự hành Context-Aware theo đúng thứ tự để đảm bảo các thành phần (Raspberry Pi, Jetson, Laptop) kết nối và đồng bộ thành công.

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
ros2 launch wheeltec_twist_mux twist_mux.launch.py jetson_ip:=25.12.4.100
```
*(Lưu ý: File launch này sẽ tự động gọi luôn node `context_aware_bridge`, mở cổng ZMQ 5555 và 5560 để sẵn sàng đón tín hiệu từ Jetson)*

### Terminal 3: Bật Cầu nối SCADA (Tùy chọn nếu dùng Web)
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 run wheeltec_scada_bridge bridge_node
```

---

## 2. Jetson Orin Nano (AI Core)
**Vai trò:** Xử lý ảnh từ Realsense, chạy mô hình YOLO/Intent CNN và tính toán lệnh điều hướng thông minh.
**IP Mặc định:** `25.12.4.100`

⚠️ **Lưu ý:** Bắt buộc phải khởi động bước này **SAU KHI** Raspberry Pi đã chạy Terminal 2 (đã mở cổng ZMQ), nếu không Jetson có thể báo lỗi không tìm thấy socket.

Mở terminal SSH vào Jetson:

### Terminal 1: Khởi chạy AI Pipeline
```bash
cd ~/context-aware  # Thay bằng đường dẫn tới repo AI trên Jetson của bạn
make jetson-up
```
*(Quá trình này sẽ bật Camera Realsense, load các mô hình AI, và bắt đầu xuất lệnh tốc độ gửi qua mạng LAN tới Pi)*

Bạn có thể xem log bằng lệnh:
```bash
make jetson-logs
```

---

## 3. Laptop (Giám sát & Điều khiển - Web SCADA)
**Vai trò:** Trạm điều khiển mặt đất (Ground Control Station), hiển thị bản đồ, telemetry, camera AI và nút dừng khẩn cấp.

Mở terminal trên máy tính Windows/Mac của bạn (tại thư mục mã nguồn project):

### Khởi chạy Web bằng Docker
```bash
cd website
docker compose up -d
```
*(Lệnh này sẽ tự động khởi tạo và chạy cả Backend (FastAPI) và Frontend (Next.js) trong container)*

### Giao diện Web:
Mở trình duyệt (Chrome/Edge) và truy cập: **`http://localhost:3000`**

---

## 🛑 Cơ chế an toàn (Arbitration)
Khi cả hệ thống hoạt động, mạch phân luồng `twist_mux` trên Raspberry Pi sẽ tự động quyết định xem robot sẽ nghe lệnh của ai theo thứ tự ưu tiên:
1. **Mức 10 (Cao nhất):** Bàn phím máy tính hoặc Joypad trên Web (`/cmd_vel_keyboard`). Khi bạn bấm phím, AI sẽ lập tức bị ghi đè.
2. **Mức 5:** Lệnh của Jetson AI (`/cmd_vel_context`). Nếu bạn thả tay khỏi bàn phím, AI sẽ chiếm quyền điều khiển.
3. **Mức 1 (Thấp nhất):** Điều hướng tự động Nav2 (`/cmd_vel_nav`).
