# Kế hoạch: Tuần tra tự động theo lịch trình với Coverage Planner

## Tổng quan

Sau khi quét map xong, robot sẽ tự động di chuyển giám sát toàn bộ map theo lịch trình:

- **Tần suất**: Mỗi 30 phút thực hiện 1 lần
- **Số vòng**: Mỗi lần đi 5 vòng
- **Điểm xuất phát/kết thúc**: Home position
- **Tuyến đường**: Tự động sinh bằng coverage planner (boustrophedon hoặc spiral)

## Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend (Next.js/React)                  │
├─────────────────────────────────────────────────────────────┤
│  • CoverageGenerator: Sinh tuyến bao phủ toàn map          │
│  • PatrolPanel: Cấu hình lịch trình và giám sát runtime    │
│  • HomePoint: Quản lý vị trí home                           │
└─────────────────────────────────────────────────────────────┘
                              ↓ REST API
┌─────────────────────────────────────────────────────────────┐
│                  Backend (FastAPI + SQLite)                  │
├─────────────────────────────────────────────────────────────┤
│  • CoverageService: Sinh và lưu coverage route             │
│  • PatrolService: Quản lý lịch trình và thực thi mission   │
│    - scheduler_loop(): Kiểm tra điều kiện mỗi 10s          │
│    - execute_patrol_mission(): Điều khiển Nav2             │
│  • Database: Lưu routes, schedules, missions, sessions      │
└─────────────────────────────────────────────────────────────┘
                              ↓ ZMQ
┌─────────────────────────────────────────────────────────────┐
│              ROS2 Bridge Node (wheeltec_scada_bridge)        │
├─────────────────────────────────────────────────────────────┤
│  • CoveragePlanner: Thuật toán boustrophedon/spiral        │
│  • Nav2 Client: Gửi waypoint goals                         │
│  • Map Subscriber: Nhận occupancy grid                     │
└─────────────────────────────────────────────────────────────┘
                              ↓ ROS2 Topics
┌─────────────────────────────────────────────────────────────┐
│                      Nav2 Stack + SLAM                       │
├─────────────────────────────────────────────────────────────┤
│  • /map: Occupancy grid từ SLAM                            │
│  • /navigate_to_pose: Action server nhận goals             │
│  • /odom: Odometry feedback                                │
└─────────────────────────────────────────────────────────────┘
```

## Luồng hoạt động

### 1. Khởi tạo (Sau khi quét map)

```
Người dùng → Coverage Generator UI
  ↓
  1. Chọn pattern: boustrophedon (lawnmower) hoặc spiral
  2. Cấu hình robot_width: 0.5m (mặc định)
  3. Cấu hình overlap: 0.1m (mặc định)
  ↓
POST /api/robot/coverage/generate
  ↓
CoverageService.generate_coverage_route()
  ↓ ZMQ command
ROS2 CoveragePlanner.generate_coverage_waypoints()
  ↓
  • Đọc occupancy grid từ /map
  • Inflate obstacles (clearance 0.3m)
  • Sinh waypoints theo pattern
  • Validate waypoints (trong bounds, free space)
  • Optimize (loại bỏ collinear points)
  ↓
Trả về waypoints → Backend
  ↓
PatrolService.save_route()
  ↓
Lưu vào database: patrol_routes table
  ↓
Frontend nhận route_id → Hiển thị thành công
```

### 2. Cấu hình lịch trình

```
Người dùng → Patrol Panel UI
  ↓
  1. Enable schedule: ON
  2. Interval minutes: 30
  3. Loops per run: 5
  ↓
POST /api/robot/patrol/schedule
  ↓
PatrolService.update_schedule()
  ↓
Lưu vào database: patrol_schedules table
  • enabled: true
  • interval_minutes: 30
  • loops_per_run: 5
  • route_id: <coverage_route_id>
  • next_trigger_at: now + 30 minutes
```

### 3. Thực thi tự động

```
Backend startup → PatrolService.scheduler_loop()
  ↓
Mỗi 10 giây kiểm tra:
  ↓
  1. Schedule có enabled?
  2. Đã đến next_trigger_at?
  3. Robot có đang idle?
  4. Robot có kết nối?
  ↓
Nếu tất cả điều kiện OK:
  ↓
PatrolService.execute_patrol_mission()
  ↓
  1. Tạo patrol_missions record (status: starting)
  2. Cập nhật patrol_runtime:
     - status: starting
     - current_loop: 0
     - total_loops: 5
     - current_waypoint_index: -1
  ↓
Với mỗi loop (1 → 5):
  ↓
  Với mỗi waypoint trong route:
    ↓
    1. Cập nhật runtime: current_waypoint_index
    2. Gửi ZMQ command: navigate_to_pose
       ↓ ROS2 Bridge
    3. Nav2 Client gửi goal → /navigate_to_pose
    4. Chờ goal complete (timeout 300s)
    5. Kiểm tra kết quả:
       - SUCCESS → Tiếp tục waypoint tiếp theo
       - FAILED → Retry 3 lần, sau đó abort mission
  ↓
  Sau khi hoàn thành tất cả waypoints:
    ↓
    1. Cập nhật runtime: current_loop += 1
    2. Nếu current_loop < total_loops → Tiếp tục loop tiếp theo
  ↓
Sau khi hoàn thành tất cả loops:
  ↓
  1. Cập nhật runtime: status = returning_home
  2. Gửi goal về home position
  3. Chờ về đến home
  ↓
  1. Cập nhật mission: status = completed
  2. Cập nhật runtime: status = idle
  3. Cập nhật schedule: next_trigger_at = now + 30 minutes
```

### 4. Giám sát runtime

```
Frontend → WebSocket connection
  ↓
Backend broadcast mỗi giây:
  {
    "patrol_status": {
      "route": { id, name, waypoint_count },
      "schedule": { enabled, interval_minutes, loops_per_run, next_trigger_at },
      "runtime": {
        "status": "running",
        "current_loop": 2,
        "total_loops": 5,
        "current_waypoint_index": 15,
        "total_waypoints": 50,
        "message": "Navigating to waypoint 16/50"
      },
      "latest_run": { id, status, started_at, completed_at }
    }
  }
  ↓
PatrolPanel hiển thị real-time:
  • Runtime status: Running
  • Current loop: 2 / 5
  • Waypoint: 16 / 50
  • Next trigger: 2026-04-08 16:15:00
  • Latest mission: #123 · completed
```

## Cấu trúc database

### patrol_routes

```sql
CREATE TABLE patrol_routes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    waypoints TEXT NOT NULL,  -- JSON array: [{"x": 1.0, "y": 2.0, "yaw": 0.0}, ...]
    created_at TEXT NOT NULL,
    session_id INTEGER,
    FOREIGN KEY (session_id) REFERENCES sessions(id)
);
```

### patrol_schedules

```sql
CREATE TABLE patrol_schedules (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    route_id INTEGER,
    enabled INTEGER NOT NULL DEFAULT 0,
    interval_minutes INTEGER NOT NULL DEFAULT 30,
    loops_per_run INTEGER NOT NULL DEFAULT 5,
    next_trigger_at TEXT,
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL,
    FOREIGN KEY (route_id) REFERENCES patrol_routes(id)
);
```

### patrol_runtime

```sql
CREATE TABLE patrol_runtime (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    status TEXT NOT NULL DEFAULT 'idle',  -- idle, starting, running, returning_home, stopped
    current_loop INTEGER NOT NULL DEFAULT 0,
    total_loops INTEGER NOT NULL DEFAULT 0,
    current_waypoint_index INTEGER NOT NULL DEFAULT -1,
    total_waypoints INTEGER NOT NULL DEFAULT 0,
    message TEXT,
    updated_at TEXT NOT NULL
);
```

### patrol_missions

```sql
CREATE TABLE patrol_missions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    route_id INTEGER,
    status TEXT NOT NULL,  -- starting, running, completed, failed, stopped
    started_at TEXT NOT NULL,
    completed_at TEXT,
    error_message TEXT,
    session_id INTEGER,
    FOREIGN KEY (route_id) REFERENCES patrol_routes(id),
    FOREIGN KEY (session_id) REFERENCES sessions(id)
);
```

## Thuật toán Coverage Planner

### Boustrophedon (Lawnmower)

```python
def _generate_boustrophedon(self, map_msg: OccupancyGrid) -> List[Dict[str, float]]:
    """
    Thuật toán:
    1. Inflate obstacles với clearance 0.3m
    2. Tính spacing giữa các hàng: robot_width - overlap
    3. Quét từ trái sang phải, từ dưới lên trên
    4. Mỗi hàng:
       - Hàng chẵn: quét từ trái → phải
       - Hàng lẻ: quét từ phải → trái (zigzag)
    5. Chỉ thêm waypoint nếu cell là free space
    6. Optimize: loại bỏ waypoints thẳng hàng (collinear)
    """
    waypoints = []
    spacing = self.robot_width - self.overlap
    num_rows = int(height / spacing)

    for row_idx in range(num_rows):
        y = row_idx * spacing
        if row_idx % 2 == 0:
            # Quét từ trái sang phải
            for x in range(0, width, spacing):
                if is_free_space(x, y):
                    waypoints.append({"x": x, "y": y, "yaw": 0.0})
        else:
            # Quét từ phải sang trái
            for x in range(width, 0, -spacing):
                if is_free_space(x, y):
                    waypoints.append({"x": x, "y": y, "yaw": 3.14})

    return optimize_waypoints(waypoints)
```

### Spiral

```python
def _generate_spiral(self, map_msg: OccupancyGrid) -> List[Dict[str, float]]:
    """
    Thuật toán:
    1. Bắt đầu từ trung tâm map
    2. Di chuyển theo hình xoắn ốc ra ngoài
    3. Hướng: phải → xuống → trái → lên → phải (lặp lại)
    4. Mỗi vòng tăng khoảng cách: robot_width - overlap
    5. Dừng khi chạm biên map hoặc obstacle
    """
    waypoints = []
    center_x, center_y = width / 2, height / 2
    spacing = self.robot_width - self.overlap

    x, y = center_x, center_y
    direction = 0  # 0: right, 1: down, 2: left, 3: up
    steps = 1

    while in_bounds(x, y):
        for _ in range(2):  # Mỗi vòng có 2 cạnh cùng độ dài
            for _ in range(steps):
                if is_free_space(x, y):
                    yaw = direction * (3.14 / 2)
                    waypoints.append({"x": x, "y": y, "yaw": yaw})

                # Di chuyển theo hướng hiện tại
                if direction == 0: x += spacing
                elif direction == 1: y -= spacing
                elif direction == 2: x -= spacing
                elif direction == 3: y += spacing

            direction = (direction + 1) % 4

        steps += 1

    return optimize_waypoints(waypoints)
```

## Validation và Safety

### 1. Parameter Validation

```python
# CoverageService.generate_coverage_route()
if not (0.1 <= robot_width <= 2.0):
    raise ValueError("robot_width must be between 0.1 and 2.0 meters")
if not (0.0 <= overlap <= 0.5):
    raise ValueError("overlap must be between 0.0 and 0.5 meters")
if pattern not in ["boustrophedon", "spiral"]:
    raise ValueError("pattern must be 'boustrophedon' or 'spiral'")
```

### 2. Map Availability

```python
# ROS2 node.py
if self._last_map_msg is None:
    return {"status": "error", "message": "No map available"}
```

### 3. Waypoint Validation

```python
# CoveragePlanner.validate_waypoints()
for wp in waypoints:
    # Kiểm tra trong bounds
    if not (0 <= wp["x"] < width and 0 <= wp["y"] < height):
        return False, f"Waypoint out of bounds: {wp}"

    # Kiểm tra trong free space (sau khi inflate obstacles)
    if not is_free_space(wp["x"], wp["y"]):
        return False, f"Waypoint in obstacle: {wp}"

    # Kiểm tra giá trị hợp lệ
    if not all(math.isfinite(v) for v in [wp["x"], wp["y"], wp["yaw"]]):
        return False, f"Invalid waypoint values: {wp}"
```

### 4. Waypoint Limit

```python
# CoverageService.generate_coverage_route()
if len(waypoints) > 1000:
    raise ValueError("Generated route exceeds maximum 1000 waypoints")
```

### 5. Obstacle Clearance

```python
# CoveragePlanner._inflate_obstacles()
clearance_meters = 0.3  # 30cm clearance
inflation_cells = int(clearance_meters / resolution)
inflated_grid = binary_dilation(grid, iterations=inflation_cells)
```

### 6. Scheduler Safety Checks

```python
# PatrolService.scheduler_loop()
if not schedule.enabled:
    continue
if datetime.now() < schedule.next_trigger_at:
    continue
if runtime.status != "idle":
    continue
if not is_robot_connected():
    continue
```

### 7. Navigation Timeout

```python
# PatrolService.execute_patrol_mission()
timeout = 300  # 5 minutes per waypoint
result = await send_nav_goal(waypoint, timeout=timeout)
if result != "SUCCESS":
    retry_count += 1
    if retry_count >= 3:
        abort_mission("Navigation failed after 3 retries")
```

## API Endpoints

### Coverage Generation

```
POST /api/robot/coverage/generate
Body: {
  "name": "Coverage_boustrophedon_20260408_153000",  // Optional, auto-generated
  "robot_width": 0.5,
  "overlap": 0.1,
  "pattern": "boustrophedon"  // or "spiral"
}
Response: {
  "route": {
    "id": 1,
    "name": "Coverage_boustrophedon_20260408_153000",
    "waypoints": [...],
    "created_at": "2026-04-08T15:30:00"
  }
}
```

### Schedule Configuration

```
POST /api/robot/patrol/schedule
Body: {
  "enabled": true,
  "interval_minutes": 30,
  "loops_per_run": 5,
  "route_id": 1
}
Response: {
  "schedule": {
    "id": 1,
    "enabled": true,
    "interval_minutes": 30,
    "loops_per_run": 5,
    "route_id": 1,
    "next_trigger_at": "2026-04-08T16:00:00"
  }
}
```

### Manual Start

```
POST /api/robot/patrol/start
Response: {
  "mission": {
    "id": 123,
    "status": "starting",
    "started_at": "2026-04-08T15:45:00"
  }
}
```

### Manual Stop

```
POST /api/robot/patrol/stop
Body: {
  "reason": "Stopped from navigation UI"
}
Response: {
  "mission": {
    "id": 123,
    "status": "stopped",
    "completed_at": "2026-04-08T15:50:00"
  }
}
```

## Testing Checklist

### Unit Tests

- [ ] CoveragePlanner.generate_coverage_waypoints() với map giả
- [ ] CoveragePlanner.\_inflate_obstacles() với các obstacle patterns
- [ ] CoveragePlanner.validate_waypoints() với waypoints hợp lệ/không hợp lệ
- [ ] CoverageService.generate_coverage_route() với các parameters khác nhau
- [ ] PatrolService.update_schedule() với enabled/disabled
- [ ] PatrolService.execute_patrol_mission() với route giả

### Integration Tests

- [ ] POST /api/robot/coverage/generate → Kiểm tra route được lưu vào DB
- [ ] POST /api/robot/patrol/schedule → Kiểm tra schedule được cập nhật
- [ ] POST /api/robot/patrol/start → Kiểm tra mission được tạo
- [ ] POST /api/robot/patrol/stop → Kiểm tra mission được dừng
- [ ] WebSocket → Kiểm tra patrol_status được broadcast

### E2E Tests

- [ ] Quét map bằng SLAM
- [ ] Sinh coverage route từ UI
- [ ] Cấu hình schedule: 30 phút, 5 vòng
- [ ] Chờ scheduler tự động trigger
- [ ] Giám sát runtime qua WebSocket
- [ ] Kiểm tra robot di chuyển đúng waypoints
- [ ] Kiểm tra robot về home sau khi hoàn thành
- [ ] Kiểm tra next_trigger_at được cập nhật
- [ ] Test manual start/stop

### Performance Tests

- [ ] Map lớn (100x100m) → Thời gian sinh route < 5s
- [ ] Route 500 waypoints → Thời gian thực thi < 2 giờ
- [ ] Scheduler loop overhead < 100ms
- [ ] WebSocket broadcast latency < 50ms

## Troubleshooting

### Coverage route không sinh được

- Kiểm tra map đã load chưa: `ros2 topic echo /map --once`
- Kiểm tra ZMQ connection: Backend logs có "ZMQ Background Listener connected"
- Kiểm tra parameters: robot_width, overlap trong khoảng hợp lệ
- Kiểm tra map có free space đủ lớn không

### Scheduler không trigger

- Kiểm tra schedule.enabled = true
- Kiểm tra next_trigger_at đã qua chưa
- Kiểm tra runtime.status = "idle"
- Kiểm tra robot connection: WebSocket có telemetry không
- Kiểm tra backend logs: "Patrol scheduler checking..."

### Navigation thất bại

- Kiểm tra Nav2 stack đang chạy: `ros2 node list | grep nav2`
- Kiểm tra goal có hợp lệ không: Trong bounds, free space
- Kiểm tra robot có stuck không: Odometry có thay đổi không
- Kiểm tra timeout: Tăng timeout nếu map lớn
- Kiểm tra retry count: Có retry đủ 3 lần chưa

### Robot không về home

- Kiểm tra home position đã set chưa: GET /api/robot/home
- Kiểm tra home position hợp lệ: Trong bounds, free space
- Kiểm tra navigation về home: Logs có "Returning to home" không

## Kết luận

Hệ thống tuần tra tự động đã được triển khai đầy đủ với các tính năng:

✅ **Coverage Planner**: Sinh tuyến bao phủ toàn map tự động
✅ **Scheduler**: Thực thi theo lịch trình (30 phút, 5 vòng)
✅ **Runtime Monitoring**: Giám sát real-time qua WebSocket
✅ **Safety Checks**: Validation, obstacle clearance, timeout, retry
✅ **Database Persistence**: Lưu routes, schedules, missions
✅ **Manual Control**: Start/stop thủ công khi cần

Hệ thống sẵn sàng để test và deploy.
