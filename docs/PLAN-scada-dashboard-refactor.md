# SCADA Dashboard Refactor — Industrial-Grade + Structured Logging

## Mục tiêu

Refactor dashboard `/dashboard` từ trạng thái hiện tại (chỉ hiển thị telemetry cơ bản, không lưu log, API stub rỗng) thành một **SCADA Dashboard chuẩn công nghiệp** với:

1. **Hệ thống Log phân loại rõ ràng** — 5 category, lưu vào SQLite, giữ vĩnh viễn
2. **Dashboard hiển thị đầy đủ thông tin** — KPI bar, charts, log viewer, system status
3. **Session management** — theo dõi phiên vận hành robot

## Thực trạng hiện tại

| Component | Trạng thái |
|---|---|
| `robot-status.tsx` | Chỉ hiển thị odom/imu raw numbers |
| `system-health.tsx` | Hiển thị ZMQ/power/DDS status cơ bản |
| `odom-chart.tsx` | Chart velocity (linear X vs angular Z) — OK |
| `analytics.py` (API) | **Stub rỗng** — trả về `[]` |
| `robot.py` (API) | **Stub rỗng** — trả về `"online"` |
| `db/models.py` | Schema có `maps`, `sessions`, `telemetry` — **không table logs** |
| `db/queries.py` | Chỉ có query maps — **không có query logs/sessions** |
| `ws/handler.py` | Forward telemetry qua WS — **không lưu DB** |

---

## User Review Required

> [!IMPORTANT]
> **Retention Policy**: Giữ tất cả log vĩnh viễn (không auto-purge). Database SQLite sẽ phát triển liên tục.

> [!IMPORTANT]
> **Telemetry Downsample**: Lưu telemetry snapshot mỗi **5 giây** (thay vì 100ms). Tạo ~17,280 rows/ngày — hợp lý cho SQLite.

> [!WARNING]
> **Navigation logs**: Focus chính là category NAVIGATION. Các category khác (POWER, COMMAND, SYSTEM, TELEMETRY) vẫn log nhưng NAVIGATION là ưu tiên hiển thị trên dashboard.

---

## Proposed Changes

### Component 1: Database Layer (Server)

Mở rộng schema SQLite + thêm queries cho log system.

#### [MODIFY] [models.py](file:///d:/wheeltec_ros2/website/server/app/db/models.py)

Thêm table `event_logs`:

```sql
CREATE TABLE IF NOT EXISTS event_logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id INTEGER REFERENCES sessions(id),
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    category TEXT NOT NULL,          -- NAVIGATION | POWER | COMMAND | SYSTEM | TELEMETRY
    severity TEXT DEFAULT 'INFO',    -- INFO | WARNING | ERROR | CRITICAL
    event_type TEXT NOT NULL,        -- nav_goal_sent, nav_goal_reached, e_stop, voltage_low...
    message TEXT NOT NULL,
    metadata TEXT                    -- JSON string for extra data
);
CREATE INDEX IF NOT EXISTS idx_logs_category ON event_logs(category);
CREATE INDEX IF NOT EXISTS idx_logs_timestamp ON event_logs(timestamp);
CREATE INDEX IF NOT EXISTS idx_logs_session ON event_logs(session_id);
```

Thêm table `telemetry_snapshots` (downsampled):

```sql
CREATE TABLE IF NOT EXISTS telemetry_snapshots (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id INTEGER REFERENCES sessions(id),
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    pos_x REAL, pos_y REAL, yaw REAL,
    vel_x REAL, vel_y REAL, vel_z REAL,
    voltage REAL,
    imu_ax REAL, imu_ay REAL, imu_az REAL,
    charging BOOLEAN DEFAULT 0
);
```

---

#### [NEW] [log_service.py](file:///d:/wheeltec_ros2/website/server/app/services/log_service.py)

Service tập trung cho logging. Tất cả log đi qua đây:

```python
class LogService:
    CATEGORIES = ["NAVIGATION", "POWER", "COMMAND", "SYSTEM", "TELEMETRY"]
    SEVERITIES = ["INFO", "WARNING", "ERROR", "CRITICAL"]
    
    async def log_event(category, event_type, message, severity="INFO", metadata=None, session_id=None)
    async def get_logs(category=None, severity=None, limit=100, offset=0, session_id=None)
    async def get_log_stats()  # Count by category, severity
```

Event types phân loại rõ ràng:

| Category | Event Types |
|---|---|
| **NAVIGATION** | `nav_goal_sent`, `nav_goal_reached`, `nav_goal_failed`, `path_planned`, `path_replanned` |
| **POWER** | `voltage_low`, `voltage_critical`, `charging_started`, `charging_stopped`, `battery_ok` |
| **COMMAND** | `cmd_vel_sent`, `emergency_stop`, `slam_control`, `map_resend` |
| **SYSTEM** | `ws_connected`, `ws_disconnected`, `zmq_link_up`, `zmq_link_down`, `session_started`, `session_ended` |
| **TELEMETRY** | `snapshot_saved` (every 5s, silent — không hiển thị trên live log) |

---

#### [NEW] [session_service.py](file:///d:/wheeltec_ros2/website/server/app/services/session_service.py)

Quản lý session (phiên vận hành):

```python
class SessionService:
    async def start_session() -> int         # Returns session_id
    async def end_session(session_id: int)   # Mark ended_at
    async def get_current_session()          # Active session or None
    async def get_all_sessions(limit, offset)
    async def update_session_stats(session_id, distance, max_speed, avg_voltage)
```

---

#### [NEW] [telemetry_service.py](file:///d:/wheeltec_ros2/website/server/app/services/telemetry_service.py)

Lưu telemetry downsampled + tính toán aggregates:

```python
class TelemetryService:
    _last_save_time = 0
    SAVE_INTERVAL = 5  # seconds
    
    async def maybe_save_snapshot(telemetry_data, session_id)  # Only saves if 5s elapsed
    async def get_snapshots(session_id=None, limit=100)
    async def get_voltage_history(hours=24)
    async def get_distance_traveled(session_id)
```

---

#### [MODIFY] [queries.py](file:///d:/wheeltec_ros2/website/server/app/db/queries.py)

Thêm raw queries cho logs, sessions, telemetry_snapshots.

---

### Component 2: API Routes (Server)

#### [MODIFY] [analytics.py](file:///d:/wheeltec_ros2/website/server/app/routes/analytics.py)

Từ stub rỗng → full API:

```
GET  /api/analytics/sessions                 — List sessions (paginated)
GET  /api/analytics/sessions/current         — Current active session
GET  /api/analytics/sessions/{id}/stats      — Session statistics
GET  /api/analytics/voltage-history?hours=24 — Voltage timeline data
GET  /api/analytics/distance?session_id=     — Total distance
GET  /api/analytics/summary                  — Dashboard summary (KPIs)
```

---

#### [NEW] [logs.py](file:///d:/wheeltec_ros2/website/server/app/routes/logs.py)

Dedicated log API:

```
GET  /api/logs?category=&severity=&limit=&offset=&session_id=  — Filtered logs
GET  /api/logs/stats        — Count by category + severity
GET  /api/logs/latest?n=50  — Latest N logs (for live feed)
```

---

#### [MODIFY] [robot.py](file:///d:/wheeltec_ros2/website/server/app/routes/robot.py)

Từ stub → real status:

```
GET  /api/robot/status  — Real connection state, last telemetry, uptime
```

---

#### [MODIFY] [main.py](file:///d:/wheeltec_ros2/website/server/app/main.py)

- Import và register logs router
- Auto-start session khi app khởi động
- Auto-end session khi app shutdown

---

### Component 3: WebSocket Handler (Server)

#### [MODIFY] [handler.py](file:///d:/wheeltec_ros2/website/server/app/ws/handler.py)

Thêm logging vào các event:

1. **On WS connect**: log `SYSTEM/ws_connected`
2. **On WS disconnect**: log `SYSTEM/ws_disconnected`
3. **On cmd_vel**: log `COMMAND/cmd_vel_sent` (rate-limited, max 1 log per 2s)
4. **On nav_goal**: log `NAVIGATION/nav_goal_sent` với coordinates
5. **On slam_control**: log `COMMAND/slam_control`
6. **Telemetry received**: call `telemetry_service.maybe_save_snapshot()` (every 5s)
7. **Voltage checks**: khi voltage < 10.5V → log `POWER/voltage_low`; < 9.5V → `POWER/voltage_critical`
8. **Forward log events to WS**: gửi `log_event` message qua WS cho client hiển thị real-time

---

### Component 4: Frontend Dashboard (Client)

Refactor hoàn toàn trang `/dashboard` theo chuẩn Industrial SCADA.

#### [MODIFY] [dashboard/page.tsx](file:///d:/wheeltec_ros2/website/client/src/app/(default)/dashboard/page.tsx)

Layout mới:

```
┌──────────────────────────────────────────────────────────────┐
│  KPI OVERVIEW BAR (4 cards)                                  │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │ Voltage  │ │ Speed    │ │ Distance │ │ Uptime   │       │
│  │ 12.4V ▲  │ │ 0.2 m/s  │ │ 142.3m   │ │ 02:34:12 │       │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │
├──────────────────────────┬───────────────────────────────────┤
│  VELOCITY CHART          │  VOLTAGE CHART (24h history)     │
│  (existing odom-chart)   │  (new - from DB snapshots)       │
├──────────────────────────┴───────────────────────────────────┤
│  NAVIGATION EVENT LOG (full-width table)                     │
│  ┌─────┬──────────┬───────────┬────────────┬────────────────┤
│  │ SEV │ TIME     │ CATEGORY  │ EVENT      │ MESSAGE        │
│  │ ● I │ 14:32:01 │ NAV       │ goal_sent  │ Goal: (2.3,1) │
│  │ ● W │ 14:31:45 │ POWER     │ volt_low   │ Battery 10.2V │
│  │ ● I │ 14:30:12 │ SYSTEM    │ ws_connect │ Client joined │
│  └─────┴──────────┴───────────┴────────────┴────────────────┘
├──────────────────────────┬───────────────────────────────────┤
│  SYSTEM HEALTH           │  SESSION INFO                    │
│  (existing, enhanced)    │  (new - current session stats)   │
└──────────────────────────┴───────────────────────────────────┘
```

---

#### [NEW] [kpi-bar.tsx](file:///d:/wheeltec_ros2/website/client/src/components/scada/kpi-bar.tsx)

4 KPI cards industrial style:

1. **Battery Voltage** — Gauge với color coding (green/orange/red), trend arrow
2. **Current Speed** — Live velocity magnitude `√(vx² + vy²)`
3. **Distance Traveled** — Từ session API, tích lũy
4. **Session Uptime** — Timer từ session start

Design: Dark card với accent glow, monospace numbers, severity-based border colors.

---

#### [NEW] [voltage-chart.tsx](file:///d:/wheeltec_ros2/website/client/src/components/scada/voltage-chart.tsx)

Area chart hiển thị voltage history 24h từ DB. Dùng `recharts`.
- Fetch từ `/api/analytics/voltage-history`
- Color zones: green (>11V), orange (10-11V), red (<10V)
- Refresh mỗi 30s

---

#### [NEW] [event-log-table.tsx](file:///d:/wheeltec_ros2/website/client/src/components/scada/event-log-table.tsx)

Industrial-grade event log viewer:

- **Table columns**: Severity dot, Timestamp, Category badge, Event type, Message
- **Category filter tabs**: ALL | NAVIGATION | POWER | COMMAND | SYSTEM
- **Severity filter**: INFO | WARNING | ERROR | CRITICAL
- **Live tail mode**: Auto-scroll, new logs appear at top
- **Paginated mode**: Toggle để tìm kiếm log cũ
- Color coding:
  - NAVIGATION → `--status-blue`
  - POWER → `--status-orange`
  - COMMAND → `--status-green`
  - SYSTEM → `--chart-2` (neutral gray)
  - ERROR/CRITICAL → `--destructive`

---

#### [NEW] [session-info.tsx](file:///d:/wheeltec_ros2/website/client/src/components/scada/session-info.tsx)

Current session card:
- Session ID, started at
- Total distance, max speed, avg voltage
- Emergency stops count
- Elapsed time (live counter)

---

#### [MODIFY] [system-health.tsx](file:///d:/wheeltec_ros2/website/client/src/components/scada/system-health.tsx)

Enhance với:
- **Battery gauge** (progress bar thay vì chỉ text "12.4V")
- **Charging indicator** animation
- Percentage estimate based on voltage range

---

#### [MODIFY] [robot-status.tsx](file:///d:/wheeltec_ros2/website/client/src/components/scada/robot-status.tsx)

Xóa mock data, thêm:
- **Map Pose** (x, y, yaw từ TF)
- **Velocity magnitude** tính toán
- Format số đẹp hơn (units: m, m/s, rad/s, m/s²)

---

### Component 5: Frontend API Hooks (Client)

#### [NEW] [use-dashboard-data.ts](file:///d:/wheeltec_ros2/website/client/src/hooks/use-dashboard-data.ts)

Custom hooks fetch data từ API:

```typescript
function useDashboardData() {
  // Fetch: /api/analytics/summary → KPIs
  // Fetch: /api/analytics/voltage-history → chart data
  // Fetch: /api/analytics/sessions/current → session info
  // Polling interval: 10s for KPIs, 30s for voltage chart
}

function useEventLogs(category?: string, severity?: string) {
  // Fetch: /api/logs/latest?n=50
  // Also listen to WS log_event for real-time updates
  // Append new logs to list without re-fetching
}
```

---

### Component 6: Navigation & Misc

#### [MODIFY] [sidebar.tsx](file:///d:/wheeltec_ros2/website/client/src/components/layout/sidebar.tsx)

Rename "Analytics" → "SCADA Monitor" để đúng mục đích.

#### [MODIFY] [ros-client.ts](file:///d:/wheeltec_ros2/website/client/src/lib/ros-client.ts)

Thêm handler cho `log_event` message type từ WS.

---

## File Change Summary

| Layer | File | Action | Mô tả |
|---|---|---|---|
| **DB** | `db/models.py` | MODIFY | Thêm `event_logs` + `telemetry_snapshots` tables |
| **DB** | `db/queries.py` | MODIFY | Thêm queries cho logs, sessions, snapshots |
| **Service** | `services/log_service.py` | NEW | Centralized logging service |
| **Service** | `services/session_service.py` | NEW | Session lifecycle management |
| **Service** | `services/telemetry_service.py` | NEW | Telemetry persistence (5s downsample) |
| **API** | `routes/analytics.py` | MODIFY | Full analytics endpoints |
| **API** | `routes/logs.py` | NEW | Log CRUD endpoints |
| **API** | `routes/robot.py` | MODIFY | Real robot status |
| **API** | `app/main.py` | MODIFY | Register routes, session lifecycle |
| **WS** | `ws/handler.py` | MODIFY | Add logging + telemetry persistence + log streaming |
| **UI** | `dashboard/page.tsx` | MODIFY | Full SCADA layout redesign |
| **UI** | `scada/kpi-bar.tsx` | NEW | 4 KPI cards |
| **UI** | `scada/voltage-chart.tsx` | NEW | Voltage history chart |
| **UI** | `scada/event-log-table.tsx` | NEW | Event log viewer with filters |
| **UI** | `scada/session-info.tsx` | NEW | Current session info card |
| **UI** | `scada/system-health.tsx` | MODIFY | Enhanced battery gauge |
| **UI** | `scada/robot-status.tsx` | MODIFY | Remove mock, add map pose |
| **Hook** | `hooks/use-dashboard-data.ts` | NEW | API data fetching hooks |
| **WS** | `lib/ros-client.ts` | MODIFY | Add log_event handler |
| **Nav** | `layout/sidebar.tsx` | MODIFY | Rename label |

**Total: 10 new files, 10 modified files**

---

## Open Questions

> [!IMPORTANT]
> **Q1**: Log table — default hiển thị **live tail** (auto-scroll, kiểu terminal) hay **paginated** (kiểu DB viewer)?
> Đề xuất: **Hybrid** — Live tail mặc định + toggle sang paginated khi cần search.

> [!IMPORTANT]
> **Q2**: Dark mode — Force dark theme cho dashboard page? SCADA công nghiệp thường dùng dark mặc định.

---

## Verification Plan

### Automated Tests

```bash
# Client build check
cd website/client && npm run build
```

### Manual Verification

1. Start server + client
2. Kết nối robot (hoặc mock telemetry)
3. Verify:
   - [ ] KPI bar hiển thị voltage, speed, distance, uptime real-time
   - [ ] Velocity chart hoạt động (existing)
   - [ ] Voltage chart hiển thị history từ DB
   - [ ] Event log table cập nhật real-time khi gửi nav_goal
   - [ ] Filter theo category NAVIGATION chỉ hiển thị navigation events
   - [ ] Session info hiển thị phiên hiện tại
   - [ ] System health có battery gauge
   - [ ] DB có dữ liệu trong `event_logs` và `telemetry_snapshots`
4. Kiểm tra log không lưu lung tung:
   - [ ] `cmd_vel` rate-limited (max 1 log/2s, không flood)
   - [ ] `telemetry snapshot` chỉ mỗi 5s
   - [ ] Mỗi log có đúng category + event_type
