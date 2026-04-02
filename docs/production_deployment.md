# Production Auto-Start Deployment Guide

Để đưa toàn bộ hệ thống Robot SCADA vào chuẩn "Production State" (cắm pin bật nguồn là tự chạy, không cần cắm màn hình hay gõ command thủ công), chúng ta sẽ sử dụng công cụ `systemd` trên Ubuntu. Đây là tiêu chuẩn vàng dùng trong cả công nghiệp lẫn server Linux.

---

## 1. ROS2 Master Launch File (Đã cấu hình)
Tôi đã tạo sẵn cho bạn một tệp tin gộp toàn bộ các Node mang tên `prod_bringup.launch.py`.
File này nằm ở: `src/turn_on_wheeltec_robot/launch/prod_bringup.launch.py`

File này sẽ gọi cùng lúc:
- Toàn bộ khung gầm ROS2, Lidar, IMU, Telemetry.
- Định vị Navigation 2 + Load Bản đồ (WHEELTEC.yaml).
- Camera luồng trực tiếp phục vụ xem Live trên Website.

*Bây giờ bạn cần biên dịch (build) lại package để ROS2 nhận diện file mới:*
```bash
cd ~/wheeltec_ros2
colcon build --packages-select turn_on_wheeltec_robot
source install/setup.bash
```

---

## 2. Tạo Linux System Service cho ROS2
Dưới tư cách Root, tạo một file cấu hình background service cho tiến trình ROS2:

```bash
sudo nano /etc/systemd/system/wheeltec_ros2.service
```

Dán đoạn nội dung sau vào (Lưu ý: thay chữ `ubuntu` bằng đúng User đang chạy trên máy RPI/Jetson của bạn):

```ini
[Unit]
Description=Wheeltec ROS2 Production Bringup
After=network.target

[Service]
Type=simple
User=ubuntu
# Thay folder theo đúng Path Của Bạn
WorkingDirectory=/home/ubuntu/wheeltec_ros2
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/wheeltec_ros2/install/setup.bash && ros2 launch turn_on_wheeltec_robot prod_bringup.launch.py"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

---

## 3. Tạo Auto-Start Cho Hệ SCADA Website (Python + Node.js)
Để web quản trị cũng tự mở, chúng ta dùng PM2 (Phần mềm quản lý Process số 1 thế giới cho hệ Node & Python web).

**Bước 3.1: Cài đặt PM2**
```bash
sudo apt update
sudo apt install -y nodejs npm
sudo npm install -g pm2
```

**Bước 3.2: Chạy FastAPI & Frontend qua PM2**
Bạn cần Build ra code production trước khi đem chạy thật:

```bash
# ----- BUILD FRONTEND NEXT.JS -----
cd ~/wheeltec_ros2/website/client
npm install
npm run build

# ----- START FRONTEND -----
# Lệnh dưới đây yêu cầu PM2 chạy Frontend ở Port 3000
pm2 start npm --name "scada-ui" -- start

# ----- START BACKEND FASTAPI -----
cd ~/wheeltec_ros2/website/server
pm2 start "uvicorn app.main:app --host 0.0.0.0 --port 8000" --name "scada-backend"
```

**Bước 3.3: Ấn định cấu hình PM2 vào hệ thống Boot**
```bash
pm2 save
pm2 startup
```
*(Chạy xong lệnh startup, PM2 sẽ in ra 1 dòng `sudo env...`, bạn copy dòng đó dán vào chạy là xong. Từ nay mỗi khi bật máy, UI cũng được bật ngay lập tức).*

---

## 4. Kích hoạt và Hoàn Thành
Chạy 2 lệnh cuối cùng để kích hoạt Service ROS2:

```bash
sudo systemctl daemon-reload
sudo systemctl enable wheeltec_ros2.service
sudo systemctl start wheeltec_ros2.service
```

🎉 **XONG! TIÊU CHUẨN PRODUCTION:** 
Bên dưới gầm con xe 24V của bạn, cứ mỗi khi bật công tắc:
1. `systemd` tự nổ máy khởi động ROS2 (15 giây).
2. PM2 tự nổ máy khởi chạy Server UI và luồng cổng truyền hình ảnh.
3. Người dùng chỉ việc mở web truy cập địa chỉ IP con xe là có thể thao tác với Telemetry, Joypad, SLAM ngay lập tức.

---

## 5. Deploy Website Lên VPS (Tách ROS2 khỏi Web Server)

### Vấn đề
Khi website chạy trên VPS (ví dụ `203.0.113.50`) và Robot nằm trong mạng LAN nội bộ (ví dụ `192.168.0.100`), hai máy không nhìn thấy nhau qua Internet. FastAPI backend cần kết nối ZMQ tới Robot nhưng không có đường mạng trực tiếp.

### Kiến trúc triển khai

```
┌──────────────────────────────────────────────────────────────┐
│                        INTERNET                              │
└──────────────────────────────────────────────────────────────┘
        │                                    │
        │ Public IP: 203.0.113.50            │ WireGuard Tunnel
        │                                    │ (10.0.0.1 <--> 10.0.0.2)
        ▼                                    ▼
┌─────────────────────┐           ┌─────────────────────────┐
│       VPS           │           │     Robot (RPi/Jetson)   │
│                     │           │                         │
│  Next.js (:3000)    │           │  ROS2 Nodes (Prod)      │
│  FastAPI (:8000)    │◄─ ZMQ ──►│  ZMQ Bridge              │
│  SQLite DB          │  10.0.0.2 │  Camera Publisher        │
│  Nginx (Reverse ──► │           │  WireGuard Client        │
│    Proxy :80/443)   │           │                         │
│  WireGuard Server   │           │  IP LAN: 192.168.0.100  │
│                     │           │  IP WG:  10.0.0.2       │
│  IP WG: 10.0.0.1   │           │                         │
└─────────────────────┘           └─────────────────────────┘
```

**Nguyên lý:** WireGuard tạo một đường hầm VPN mã hóa giữa VPS và Robot. Sau khi tunnel hoạt động, VPS có thể truy cập Robot qua IP ảo `10.0.0.2` như thể chúng nằm cùng mạng LAN. FastAPI chỉ cần đổi `ROBOT_IP=10.0.0.2` là kết nối ZMQ hoạt động xuyên Internet.

---

### Bước 5.1: Cài WireGuard trên VPS

```bash
# Trên VPS (Ubuntu)
sudo apt update && sudo apt install -y wireguard

# Tạo cặp key
wg genkey | tee /etc/wireguard/server_private.key | wg pubkey > /etc/wireguard/server_public.key
chmod 600 /etc/wireguard/server_private.key

# Lấy key ra để dùng ở bước sau
SERVER_PRIVATE=$(cat /etc/wireguard/server_private.key)
SERVER_PUBLIC=$(cat /etc/wireguard/server_public.key)
```

Tạo file cấu hình WireGuard trên VPS:
```bash
sudo nano /etc/wireguard/wg0.conf
```

```ini
[Interface]
Address = 10.0.0.1/24
ListenPort = 51820
PrivateKey = <SERVER_PRIVATE_KEY>

# Cho phép forward traffic
PostUp = iptables -A FORWARD -i wg0 -j ACCEPT
PostDown = iptables -D FORWARD -i wg0 -j ACCEPT

[Peer]
# Robot
PublicKey = <ROBOT_PUBLIC_KEY>
AllowedIPs = 10.0.0.2/32
```

```bash
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0
```

---

### Bước 5.2: Cài WireGuard trên Robot

```bash
# Trên Robot (RPi/Jetson)
sudo apt update && sudo apt install -y wireguard

# Tạo cặp key
wg genkey | tee /etc/wireguard/robot_private.key | wg pubkey > /etc/wireguard/robot_public.key
chmod 600 /etc/wireguard/robot_private.key

ROBOT_PRIVATE=$(cat /etc/wireguard/robot_private.key)
ROBOT_PUBLIC=$(cat /etc/wireguard/robot_public.key)
```

Tạo file cấu hình:
```bash
sudo nano /etc/wireguard/wg0.conf
```

```ini
[Interface]
Address = 10.0.0.2/24
PrivateKey = <ROBOT_PRIVATE_KEY>

[Peer]
# VPS
PublicKey = <SERVER_PUBLIC_KEY>
Endpoint = 203.0.113.50:51820
AllowedIPs = 10.0.0.1/32
PersistentKeepalive = 25
```

> **Lưu ý:** `PersistentKeepalive = 25` giữ tunnel luôn sống ngay cả khi Robot nằm sau NAT/Router. Thay `203.0.113.50` bằng IP thật của VPS.

```bash
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0
```

**Kiểm tra kết nối:** Từ VPS chạy `ping 10.0.0.2`, từ Robot chạy `ping 10.0.0.1`. Nếu có reply → Tunnel thành công.

---

### Bước 5.3: Cấu hình ZMQ trỏ qua Tunnel

Trên Robot, ZMQ Bridge **không cần thay đổi gì** — nó vẫn bind trên `0.0.0.0` các port 5555/5556/5557 như bình thường.

Trên VPS, tạo file `.env` cho FastAPI:
```bash
cd ~/wheeltec_ros2/website/server
nano .env
```

```env
ROBOT_IP=10.0.0.2
ZMQ_CMD_PORT=5555
ZMQ_TELEMETRY_PORT=5556
ZMQ_CAMERA_PORT=5557
```

FastAPI đã đọc biến môi trường qua `pydantic_settings`, nên chỉ cần đổi file `.env` là tự động trỏ ZMQ xuyên WireGuard tunnel tới Robot.

---

### Bước 5.4: Deploy Website trên VPS

```bash
# ----- CLONE CODE LÊN VPS -----
cd ~
git clone <your-repo-url> wheeltec_ros2
cd wheeltec_ros2/website

# ----- BACKEND -----
cd server
pip install -r requirements.txt
pm2 start "uvicorn app.main:app --host 0.0.0.0 --port 8000" --name "scada-backend"

# ----- FRONTEND -----
cd ../client
# Tạo file .env.production cho Next.js trỏ về backend VPS
echo "NEXT_PUBLIC_WS_URL=wss://yourdomain.com/ws" > .env.production
npm install && npm run build
pm2 start npm --name "scada-ui" -- start

pm2 save && pm2 startup
```

---

### Bước 5.5: Cài Nginx Reverse Proxy + SSL

```bash
sudo apt install -y nginx certbot python3-certbot-nginx
sudo nano /etc/nginx/sites-available/scada
```

```nginx
server {
    listen 80;
    server_name yourdomain.com;

    # Frontend Next.js
    location / {
        proxy_pass http://127.0.0.1:3000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host $host;
        proxy_cache_bypass $http_upgrade;
    }

    # Backend FastAPI + WebSocket
    location /ws {
        proxy_pass http://127.0.0.1:8000/ws;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_read_timeout 86400;
    }

    location /api/ {
        proxy_pass http://127.0.0.1:8000/api/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }
}
```

```bash
sudo ln -s /etc/nginx/sites-available/scada /etc/nginx/sites-enabled/
sudo nginx -t && sudo systemctl reload nginx

# SSL tự động (miễn phí)
sudo certbot --nginx -d yourdomain.com
```

---

### Bước 5.6: Tóm tắt phân tách kiến trúc

| Thành phần | Chạy trên | Ghi chú |
|---|---|---|
| ROS2 Nodes (prod_bringup) | Robot | systemd auto-start |
| ZMQ Bridge | Robot | Bind 0.0.0.0:5555-5557 |
| WireGuard Client | Robot | Auto-connect tới VPS |
| WireGuard Server | VPS | Lắng nghe :51820 |
| FastAPI Backend | VPS | Kết nối ZMQ qua 10.0.0.2 |
| Next.js Frontend | VPS | PM2 auto-start |
| Nginx + SSL | VPS | Reverse proxy :80/443 |
| SQLite DB | VPS | Lưu logs, sessions, telemetry |

> **Kết quả:** Robot chỉ chạy ROS2 + ZMQ (nhẹ, tập trung xử lý realtime). Toàn bộ web, database, business logic nằm trên VPS mạnh hơn. Người dùng truy cập `https://yourdomain.com` từ bất kỳ đâu trên thế giới để điều khiển Robot.
