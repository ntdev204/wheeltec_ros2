# Kế hoạch triển khai: Robot tuần tra định kỳ sau khi quét map

## 1. Mục tiêu

Sau khi hoàn tất quét map và robot đã có map hoạt động ổn định, bổ sung chế độ **tuần tra giám sát định kỳ** với yêu cầu:

- Cứ **30 phút** khởi động **1 phiên tuần tra**.
- Mỗi phiên tuần tra đi **5 vòng** theo toàn bộ tuyến giám sát.
- Mỗi phiên **xuất phát từ Home**.
- Mỗi phiên **kết thúc tại Home**.
- Chỉ lập kế hoạch, **chưa viết code**.

---

## 2. Hiện trạng codebase liên quan

### 2.1 Giao diện điều hướng
- `website/client/src/app/(default)/navigation/page.tsx`
  - Đã có:
    - hiển thị map,
    - gửi goal thủ công,
    - quản lý Home,
    - export path CSV.
  - Chưa có:
    - cấu hình tuyến tuần tra,
    - lịch chạy định kỳ,
    - trạng thái mission tuần tra.

### 2.2 Home position
- `website/server/app/services/home_service.py`
  - Đang lưu/đọc 1 điểm Home duy nhất.
  - Đã có logic auto-return khi pin thấp.

### 2.3 Ghi nhận đường đi
- `website/server/app/services/path_service.py`
  - Đã lưu global plan / local plan / real path cho từng nav goal.
  - Phù hợp để tái sử dụng cho audit và giám sát đường tuần tra.

### 2.4 WebSocket điều khiển robot
- `website/server/app/ws/handler.py`
  - Đã xử lý:
    - `nav_goal`,
    - `set_home`,
    - `go_home`,
    - auto-return khi pin thấp.
  - Đây là vị trí quan trọng để tích hợp trạng thái mission hoặc phát sự kiện mission ra frontend.

### 2.5 Database hiện tại
- `website/server/app/db/models.py`
  - Đã có bảng:
    - `home_position`,
    - `nav_paths`,
    - `event_logs`,
    - `telemetry_snapshots`,
    - `maps`,
    - `sessions`.
  - Chưa có bảng cấu hình và lịch sử mission tuần tra.

### 2.6 ROS2 waypoint loop có sẵn
- `src/nav2_waypoint_cycle/nav2_waypoint_cycle/waypoint_cycle.py`
  - Hiện là node đơn giản dựa trên `clicked_point`.
  - Có khả năng loop waypoint nhưng chưa đạt mức production:
    - chưa có scheduler,
    - chưa có mission state machine rõ ràng,
    - chưa có persist cấu hình,
    - chưa có start/end tại Home,
    - chưa có quản lý 5 vòng / 30 phút / pause / resume / abort.

---

## 3. Phạm vi tính năng cần bổ sung

## 3.1 Chức năng chính
1. Người vận hành định nghĩa **tuyến tuần tra** trên map.
2. Hệ thống lưu tuyến đó thành một **patrol route** bền vững.
3. Hệ thống có **lịch tuần tra định kỳ**: mỗi 30 phút chạy 1 lần.
4. Mỗi lần chạy tạo ra một **patrol mission** gồm:
   - đi từ Home đến tuyến tuần tra,
   - chạy hết tuyến,
   - lặp lại đủ 5 vòng,
   - quay về Home.
5. Frontend hiển thị:
   - cấu hình tuyến,
   - trạng thái lịch,
   - vòng hiện tại,
   - waypoint hiện tại,
   - lỗi / dừng khẩn / pin thấp / quay về Home.
6. Backend ghi log đầy đủ để truy vết.

## 3.2 Không nằm trong phạm vi giai đoạn đầu
- Tự động sinh tuyến bao phủ toàn bộ map bằng coverage planner phức tạp.
- Multi-floor / multi-map patrol.
- Nhiều route chạy song song.
- Replanning nâng cao theo semantic zones.

> Giai đoạn đầu nên hiểu “di chuyển trong toàn bộ map” là **đi theo tuyến giám sát đã được người vận hành cấu hình để phủ khu vực cần giám sát**, không phải tự động coverage path toàn bản đồ.

---

## 4. Kiến trúc đề xuất

## 4.1 Nguyên tắc phân tách trách nhiệm

### Backend chịu trách nhiệm
- lưu cấu hình route,
- lưu cấu hình schedule,
- tạo mission run,
- phát lệnh start/stop tuần tra,
- quản lý trạng thái nghiệp vụ,
- phát trạng thái cho frontend,
- ghi log / audit.

### ROS2 node chịu trách nhiệm
- thực thi mission waypoint với Nav2,
- theo dõi tiến độ waypoint / vòng lặp,
- xử lý success / fail / retry / abort ở mức navigation runtime,
- trả trạng thái mission về backend.

### Frontend chịu trách nhiệm
- cấu hình route,
- bật/tắt lịch tuần tra,
- xem trạng thái mission,
- thao tác start now / stop / pause nếu cần.

---

## 4.2 Quyết định kiến trúc quan trọng

### Scheduler nên đặt ở backend, không đặt trong ROS2 node

**Lý do:**
- Backend hiện đã là nơi quản lý session, home, logs, database.
- Schedule là logic nghiệp vụ và cần persistence.
- Frontend cần đọc/sửa lịch qua API dễ hơn nếu scheduler ở backend.
- ROS2 node nên tập trung vào execution runtime, tránh gánh business scheduling.

### Mission executor nên là ROS2 node mới hoặc nâng cấp node hiện tại

Ưu tiên:
- tạo một executor rõ ràng hơn từ `src/nav2_waypoint_cycle/nav2_waypoint_cycle/waypoint_cycle.py`,
- dùng Nav2 action/client bài bản,
- nhận mission từ backend qua bridge hiện có.

Không nên dùng nguyên xi node hiện tại vì:
- phụ thuộc `clicked_point`,
- thiếu mission lifecycle,
- thiếu tích hợp Home / 5 loops / schedule / resume / observability.

---

## 5. Mô hình dữ liệu đề xuất

## 5.1 Bảng `patrol_routes`
Lưu định nghĩa tuyến tuần tra.

**Trường đề xuất:**
- `id`
- `name`
- `map_id`
- `waypoints_json` — danh sách waypoint theo thứ tự
- `is_active`
- `created_at`
- `updated_at`

**Ý nghĩa:**
- Mỗi route gắn với 1 map cụ thể.
- Waypoint phải nằm trong frame `map`.
- Route cần version rõ ràng để tránh map đổi nhưng route cũ vẫn chạy.

## 5.2 Bảng `patrol_schedules`
Lưu cấu hình lịch tuần tra.

**Trường đề xuất:**
- `id`
- `route_id`
- `enabled`
- `interval_minutes` (mặc định 30)
- `loops_per_run` (mặc định 5)
- `start_from_home` (true)
- `return_to_home` (true)
- `last_triggered_at`
- `next_trigger_at`
- `created_at`
- `updated_at`

## 5.3 Bảng `patrol_runs`
Lưu từng lần thực thi mission tuần tra.

**Trường đề xuất:**
- `id`
- `schedule_id`
- `route_id`
- `session_id`
- `status` (`pending`, `starting`, `running`, `paused`, `returning_home`, `completed`, `aborted`, `failed`)
- `current_loop`
- `total_loops`
- `current_waypoint_index`
- `started_at`
- `ended_at`
- `failure_reason`
- `started_from_home_x`
- `started_from_home_y`
- `ended_at_home` (boolean)

## 5.4 Bảng `patrol_run_events`
Lưu event chi tiết cho từng run.

**Trường đề xuất:**
- `id`
- `run_id`
- `timestamp`
- `event_type`
- `severity`
- `message`
- `metadata`

> Có thể tái sử dụng `event_logs` cho giai đoạn đầu nếu muốn giảm diff, nhưng về lâu dài nên có bảng riêng cho mission-level observability.

---

## 6. Luồng nghiệp vụ đề xuất

## 6.1 Điều kiện trước khi cho phép bật tuần tra định kỳ
Hệ thống chỉ cho enable schedule khi đồng thời thỏa:
- đã có map active,
- đã set Home,
- đã cấu hình route với ít nhất số waypoint tối thiểu,
- Nav2 đang healthy,
- robot không trong trạng thái charging,
- không có mission tuần tra khác đang chạy.

## 6.2 Luồng chạy định kỳ mỗi 30 phút
1. Backend scheduler kiểm tra `patrol_schedules`.
2. Nếu đến `next_trigger_at` và đủ điều kiện an toàn:
   - tạo `patrol_run` mới,
   - gửi lệnh start mission sang ROS runtime.
3. ROS mission executor nhận route:
   - xác nhận Home hiện tại,
   - di chuyển về Home trước khi bắt đầu nếu robot lệch Home,
   - bắt đầu vòng 1,
   - chạy hết route,
   - lặp đến đủ 5 vòng,
   - quay về Home,
   - báo completed.
4. Backend cập nhật trạng thái run và tính `next_trigger_at` mới.
5. Frontend nhận WS update để hiển thị trạng thái.

## 6.3 State machine mission đề xuất

```text
IDLE
  -> PRECHECK
  -> GO_TO_HOME_START
  -> RUN_LOOP
  -> LOOP_COMPLETED
  -> RUN_LOOP (n+1)
  -> RETURN_HOME_END
  -> COMPLETED

Các nhánh lỗi:
PRECHECK -> BLOCKED
RUN_LOOP -> RETRY_WAYPOINT
RUN_LOOP -> PAUSED
RUN_LOOP -> ABORTED
Bất kỳ state nào -> EMERGENCY_STOPPED
Bất kỳ state nào -> AUTO_RETURN_HOME
```

## 6.4 Quy tắc vòng lặp
- Một “vòng” = đi toàn bộ waypoint của route theo đúng thứ tự.
- `loops_per_run = 5`.
- Sau khi hoàn thành waypoint cuối cùng của vòng 5:
  - không tự quay lại đầu route nữa,
  - chuyển sang `RETURN_HOME_END`.

---

## 7. Thay đổi theo lớp hệ thống

## 7.1 ROS2 layer

### File liên quan
- `src/nav2_waypoint_cycle/nav2_waypoint_cycle/waypoint_cycle.py`
- có thể bổ sung package/node mới trong cùng package hoặc package mới chuyên cho patrol mission.

### Trách nhiệm cần có
- nhận danh sách waypoint + số vòng + Home,
- điều hướng từng waypoint bằng Nav2 action,
- publish mission progress,
- retry waypoint theo chính sách có giới hạn,
- abort mission khi:
  - emergency stop,
  - pin thấp tới ngưỡng auto-return,
  - mất Nav2,
  - người vận hành stop mission.

### Khuyến nghị
- Không dùng `clicked_point` làm input chính nữa.
- Chuyển sang model “mission request” rõ ràng.
- Có status topic/service/action riêng cho patrol.

---

## 7.2 Backend layer

### File likely thay đổi
- `website/server/app/db/models.py`
- `website/server/app/routes/robot.py`
- `website/server/app/ws/handler.py`
- `website/server/app/services/home_service.py`
- `website/server/app/services/path_service.py`
- cần thêm service mới cho patrol schedule/run.

### Trách nhiệm cần thêm
- CRUD route tuần tra,
- CRUD schedule,
- scheduler background task,
- start/stop patrol run,
- nhận progress từ ROS bridge,
- phát WS event cho frontend,
- ghi event log.

### Điểm tích hợp quan trọng
- `ws/handler.py` hiện đã là hub cho telemetry + command.
- Nên mở rộng để phát thêm:
  - `patrol_status`,
  - `patrol_progress`,
  - `patrol_error`,
  - `patrol_schedule_updated`.

---

## 7.3 Frontend layer

### File likely thay đổi
- `website/client/src/app/(default)/navigation/page.tsx`
- `website/client/src/components/control/home-point.tsx`
- `website/client/src/components/map/robot-map.tsx`
- có thể thêm component mới cho patrol config/status.

### Chức năng UI cần có
1. **Route Editor**
   - thêm/sửa/xóa waypoint tuần tra,
   - hiển thị thứ tự waypoint,
   - lưu route.

2. **Schedule Config**
   - bật/tắt tuần tra định kỳ,
   - interval phút,
   - số vòng mỗi run,
   - chọn route active.

3. **Mission Status Panel**
   - trạng thái hiện tại,
   - vòng hiện tại / tổng số vòng,
   - waypoint hiện tại,
   - thời điểm lần chạy kế tiếp,
   - nút start now / stop.

4. **Safety Indicators**
   - Home chưa set,
   - pin thấp,
   - robot đang charging,
   - Nav2 offline,
   - mission bị block.

---

## 8. Quy tắc an toàn và vận hành

## 8.1 Các điều kiện chặn không cho mission bắt đầu
- chưa có Home,
- route rỗng,
- map route không khớp map active,
- battery dưới ngưỡng an toàn bắt đầu tuần tra,
- robot đang manual control,
- robot đang chạy mission khác,
- E-stop đang active,
- localization quality không đạt ngưỡng.

## 8.2 Các điều kiện buộc mission dừng hoặc chuyển trạng thái
- battery chạm ngưỡng auto-return,
- mất localization,
- Nav2 action thất bại liên tiếp vượt ngưỡng retry,
- obstacle kéo dài vượt timeout,
- user stop mission,
- chuyển sang charging.

## 8.3 Quy tắc Home
- Home là điểm duy nhất dùng cho:
  - bắt đầu mission,
  - kết thúc mission,
  - auto-return.
- Nếu robot đang không ở Home khi schedule kích hoạt:
  - hệ thống phải quyết định rõ:
    1. bắt buộc quay về Home trước khi bắt đầu, hoặc
    2. reject run.

**Khuyến nghị:** luôn quay về Home trước khi bắt đầu để đúng yêu cầu nghiệp vụ.

---

## 9. Validation cần có

## 9.1 Validation route
- waypoint phải thuộc map frame,
- waypoint không vượt ngoài biên map,
- route phải có tối thiểu N điểm (ví dụ >= 2),
- khoảng cách giữa các điểm không bất hợp lý,
- route phải lưu kèm `map_id`/version.

## 9.2 Validation schedule
- `interval_minutes > 0`,
- `loops_per_run > 0`,
- chỉ 1 schedule active cho 1 robot ở giai đoạn đầu,
- không cho enable nếu route chưa hợp lệ.

## 9.3 Validation runtime
- mission request phải có route + home + loops,
- không nhận mission mới khi mission cũ chưa kết thúc,
- tránh duplicate trigger nếu backend restart gần mốc schedule.

---

## 10. Observability và logging

## 10.1 Cần log các sự kiện sau
- schedule enabled/disabled,
- route created/updated/deleted,
- patrol run created,
- mission started,
- loop started/completed,
- waypoint reached,
- waypoint retry,
- mission paused,
- mission aborted,
- battery forced return,
- mission completed and returned home.

## 10.2 Metrics nên theo dõi
- số patrol run thành công / thất bại,
- thời gian hoàn thành mỗi run,
- số waypoint fail / retry,
- phần trăm run kết thúc đúng Home,
- số lần schedule bị skip do safety block.

## 10.3 Tái sử dụng hạ tầng hiện có
- `event_logs` trong `website/server/app/db/models.py`
- `PathService` để lưu route thực tế của từng chặng
- websocket telemetry để hiển thị tiến độ trực tiếp

---

## 11. Kế hoạch triển khai theo giai đoạn

## Giai đoạn 1 — Chuẩn hóa domain tuần tra
**Mục tiêu:** có mô hình dữ liệu và luồng nghiệp vụ rõ ràng.

- Thiết kế schema cho `patrol_routes`, `patrol_schedules`, `patrol_runs`.
- Chốt JSON schema cho waypoint route.
- Chốt mission state machine.
- Quy định 1 robot / 1 schedule active ở phiên bản đầu.

**Tiêu chí xong:**
- Có schema DB rõ ràng.
- Có contract message giữa backend và ROS2 executor.
- Có rule validation và safety checklist.

## Giai đoạn 2 — Runtime executor ROS2
**Mục tiêu:** robot chạy được 1 patrol run theo route + loops + home.

- Nâng cấp hoặc thay thế `waypoint_cycle.py` thành mission executor.
- Thực thi 5 loops theo route.
- Start tại Home, end tại Home.
- Publish progress và terminal state.

**Tiêu chí xong:**
- Chạy tay 1 mission thành công.
- Quan sát được loop count, waypoint index, completion state.

## Giai đoạn 3 — Backend scheduling và persistence
**Mục tiêu:** backend tự kích hoạt mission mỗi 30 phút.

- Thêm service quản lý route/schedule/run.
- Thêm scheduler background task.
- Persist next trigger / last trigger.
- Đồng bộ state mission từ ROS2 về DB.

**Tiêu chí xong:**
- Enable schedule xong backend tự tạo run định kỳ.
- Không bị duplicate run khi restart đơn giản.

## Giai đoạn 4 — Frontend cấu hình và giám sát
**Mục tiêu:** người vận hành cấu hình và theo dõi đầy đủ trên UI.

- UI route editor.
- UI schedule config.
- UI mission status.
- cảnh báo blocked conditions.

**Tiêu chí xong:**
- Operator có thể cấu hình route, bật lịch, xem tiến độ.

## Giai đoạn 5 — Hardening production
**Mục tiêu:** an toàn và ổn định thực tế.

- retry policy có giới hạn,
- timeout / stuck detection,
- safety interlocks,
- test chạy dài,
- audit logs đầy đủ.

**Tiêu chí xong:**
- Chạy soak test nhiều chu kỳ.
- Có quy trình recovery sau restart / mất mạng / Nav2 lỗi.

---

## 12. Failure handling đề xuất

## 12.1 Thất bại tại waypoint
- Retry 1–2 lần tối đa.
- Nếu vẫn fail:
  - đánh dấu mission `failed` hoặc `aborted` theo policy,
  - quay Home nếu còn an toàn,
  - log đầy đủ nguyên nhân.

## 12.2 Backend restart
- Khi backend khởi động lại:
  - đọc `patrol_runs` còn trạng thái running/pending,
  - không tự tạo thêm run mới nếu run cũ chưa được reconcile,
  - cập nhật `next_trigger_at` cẩn thận để tránh bắn lặp.

## 12.3 ROS executor restart
- Nếu ROS executor chết giữa mission:
  - backend phải đánh dấu run ở trạng thái lỗi hoặc mất đồng bộ,
  - operator phải nhìn thấy cảnh báo rõ ràng,
  - không tự âm thầm restart mission giữa chừng ở bản đầu.

## 12.4 Pin thấp
- Ưu tiên luật hiện có trong `website/server/app/ws/handler.py`.
- Khi auto-return kích hoạt:
  - patrol run chuyển sang `returning_home` hoặc `aborted_low_battery`,
  - không tiếp tục vòng còn lại.

---

## 13. Testing plan

## 13.1 Unit test
- validation route,
- validation schedule,
- state transition của patrol run,
- tính `next_trigger_at`,
- policy retry / timeout.

## 13.2 Integration test
- backend tạo patrol run đúng lúc,
- backend không tạo trùng run,
- route/schedule CRUD,
- đồng bộ progress từ ROS2 về DB/WS.

## 13.3 Robot/system test
- robot chạy 1 route ngắn 5 vòng,
- robot luôn quay về Home cuối mission,
- pin thấp giữa mission thì auto-return,
- mất Nav2 / fail waypoint thì mission dừng đúng trạng thái.

## 13.4 Soak test
- chạy nhiều chu kỳ 30 phút mô phỏng hoặc rút ngắn interval trong môi trường test,
- kiểm tra drift trạng thái, memory leak, duplicate scheduling.

---

## 14. Rủi ro chính

## Rủi ro 1 — “Toàn bộ map” chưa được định nghĩa kỹ
Nếu không có khái niệm coverage route rõ ràng, yêu cầu dễ bị hiểu sai.

**Giải pháp:**
- Chốt rằng giai đoạn đầu dùng **route tuần tra do người vận hành cấu hình**.

## Rủi ro 2 — Scheduler và runtime không đồng bộ
Backend nghĩ mission đang chạy nhưng ROS executor đã chết.

**Giải pháp:**
- heartbeat / progress timestamp,
- timeout để phát hiện stale run.

## Rủi ro 3 — Restart gây duplicate patrol run

**Giải pháp:**
- `patrol_runs` phải có idempotency guard,
- scheduler chỉ trigger khi không có active run.

## Rủi ro 4 — Home thay đổi trong lúc schedule đang enable

**Giải pháp:**
- khi Home đổi, mission đang chạy giữ nguyên snapshot Home của run,
- run kế tiếp mới dùng Home mới.

## Rủi ro 5 — Pin thấp giữa vòng 3 hoặc vòng 4

**Giải pháp:**
- ưu tiên safety, dừng patrol và quay Home ngay.

---

## 15. File dự kiến sẽ thay đổi khi implement

### Backend
- `website/server/app/db/models.py`
- `website/server/app/routes/robot.py`
- `website/server/app/ws/handler.py`
- `website/server/app/services/home_service.py`
- `website/server/app/services/path_service.py`
- thêm mới service/module cho:
  - patrol route management,
  - patrol schedule management,
  - patrol run orchestration.

### Frontend
- `website/client/src/app/(default)/navigation/page.tsx`
- `website/client/src/components/map/robot-map.tsx`
- `website/client/src/components/control/home-point.tsx`
- thêm component mới cho:
  - patrol route editor,
  - patrol schedule panel,
  - patrol mission status.

### ROS2
- `src/nav2_waypoint_cycle/nav2_waypoint_cycle/waypoint_cycle.py`
- hoặc package/node mới chuyên `patrol_executor`.

---

## 16. Đề xuất triển khai thực tế

## Phương án khuyến nghị
**Phương án A — Backend scheduler + ROS patrol executor riêng**

Đây là phương án nên chọn vì:
- phù hợp kiến trúc hiện có,
- dễ persist và audit,
- rõ ràng ranh giới giữa business scheduling và robot execution,
- dễ mở rộng UI và log.

## Không khuyến nghị ở giai đoạn đầu
**Phương án B — nhét toàn bộ logic schedule vào ROS2 node**
- khó quản trị bằng web,
- khó lưu lịch sử nghiệp vụ,
- khó recovery sau restart backend/UI,
- khó audit production.

---

## 17. Câu hỏi cần chốt trước khi code

1. “Di chuyển trong toàn bộ map” có nghĩa là:
   - chạy theo route do người vận hành vẽ sẵn,
   - hay cần thuật toán tự coverage toàn bộ free space?

2. Khi tới mốc 30 phút mà mission cũ chưa xong thì:
   - bỏ qua lượt đó,
   - hay xếp hàng mission kế tiếp?

**Khuyến nghị:** bỏ qua lượt mới nếu mission cũ chưa hoàn tất.

3. Nếu robot đang không ở Home lúc đến lịch thì:
   - tự quay về Home rồi mới bắt đầu,
   - hay báo skip?

**Khuyến nghị:** tự quay về Home trước khi bắt đầu.

4. Có cần operator chỉnh được `30 phút` và `5 vòng` trên UI hay hard-code theo yêu cầu hiện tại?

**Khuyến nghị:** lưu cấu hình DB nhưng mặc định 30 phút / 5 vòng.

5. Có cần pause/resume mission hay chỉ cần start/stop?

**Khuyến nghị giai đoạn đầu:** start/stop, chưa làm resume.

---

## 18. Kết luận

Hướng triển khai phù hợp nhất cho codebase hiện tại là:

- **Backend** quản lý route + schedule + patrol run + logging.
- **ROS2 executor** chịu trách nhiệm chạy tuần tự waypoint/loop/Home với Nav2.
- **Frontend** cung cấp cấu hình và giám sát thời gian thực.

Để an toàn và dễ vận hành production, nên triển khai theo thứ tự:
1. chốt dữ liệu + state machine,
2. làm executor chạy 1 mission hoàn chỉnh,
3. thêm scheduler backend,
4. thêm UI cấu hình/giám sát,
5. hardening và test dài hạn.

---

## 19. Tóm tắt quyết định đề xuất

- Scheduler đặt ở backend.
- Mission execution đặt ở ROS2.
- Giai đoạn đầu dùng **route tuần tra cấu hình sẵn**, không tự coverage map.
- Mỗi 30 phút tạo 1 patrol run nếu hệ thống an toàn và không có run đang active.
- Mỗi run gồm:
  - về Home nếu cần,
  - chạy 5 vòng route,
  - quay về Home,
  - ghi log đầy đủ.
