┌─────────────────────┐ ┌──────────────────────┐
│ VPS │ │ Robot (RPi) │
│ │ │ │
│ FastAPI connects │ SSH Tunnel │ ZMQ binds │
│ to localhost:555x ◄├──────────────────┤ 0.0.0.0:555x │
│ │ (Robot khởi │ │
│ ROBOT_IP=127.0.0.1 │ tạo kết nối) │ Outbound SSH ──────►│
└─────────────────────┘ └──────────────────────┘

# Trên Robot

ssh-keygen -t ed25519 -f ~/.ssh/vps_tunnel -N ""
ssh-copy-id -i ~/.ssh/vps_tunnel.pub user@VPS_IP

sudo nano /etc/systemd/system/scada-tunnel.service

```bash
[Unit]
Description=SCADA Reverse SSH Tunnel to VPS
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=ubuntu
ExecStart=/usr/bin/ssh -N -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes -R 5555:localhost:5555 -R 5556:localhost:5556 -R 5557:localhost:5557 -i /home/ubuntu/.ssh/vps_tunnel user@VPS_IP
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

sudo systemctl enable scada-tunnel.service
sudo systemctl start scada-tunnel.service

# Trên VPS

# Tạo file .env cho FastAPI

cd ~/wheeltec_ros2/website/server
nano .env

ROBOT_IP=127.0.0.1
ZMQ_CMD_PORT=5555
ZMQ_TELEMETRY_PORT=5556
ZMQ_CAMERA_PORT=5557

# Khởi động lại FastAPI

pm2 restart scada-backend
