# PLAN: SCADA Robot Control Website

> See full implementation plan: [implementation_plan.md](file:///C:/Users/thientn204/.gemini/antigravity/brain/fd8e58e3-1347-41a0-923c-4fdf59f451f8/implementation_plan.md)

## Summary

Build a SCADA web app for Wheeltec ROS2 robot with:
- **8-direction keyboard control** (W/S/A/D/Q/E/Z/C + Space/U/I/X) — global across all pages
- **Home Page**: Control pad, camera feed, saved maps
- **Dashboard/SLAM**: Real-time OccupancyGrid, SLAM start/stop/save/delete
- **Dashboard/SCADA**: Voltage gauges, IMU data, odometry charts, system health

## Tech Stack
- **Frontend**: Next.js 16 + Tailwind + shadcn/ui + recharts (existing `website/client`)
- **Backend**: FastAPI + rclpy + aiortc + aiosqlite (new `website/server`)
- **ROS2**: Direct rclpy node — no rosbridge needed

## Key ROS2 Topics
| Topic | Type | Usage |
|---|---|---|
| `/cmd_vel` | Twist | Robot velocity control |
| `/odom` | Odometry | Position + velocity |
| `/imu/data_raw` | Imu | IMU sensor data |
| `/PowerVoltage` | Float32 | Battery voltage |
| `/map` | OccupancyGrid | SLAM map data |
| Camera | HTTP :8080 | web_video_server |

## 5 Phases, 15 Tasks
1. Backend Foundation (5 tasks)
2. Frontend Core (3 tasks)
3. Home Page (3 tasks)
4. Dashboard SLAM (2 tasks)
5. Dashboard SCADA Analytics (3 tasks)

Created: 2026-03-26
