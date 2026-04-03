# PLAN: Refactor RPP Controller cho Mecanum Robot

> **Status:** 📋 Planning  
> **Created:** 2026-04-03  
> **Branch hiện tại:** `main`  
> **Mục tiêu:** Giữ RPP (bám đường chuẩn) nhưng khắc phục 2 lỗi lớn: (1) xoay vòng tại goal, (2) không đi ngang được.

---

## 1. Phân tích Vấn đề Hiện Tại

### Vấn đề 1: Xoay vòng tại goal (Orbiting)
- **Nguyên nhân gốc:** `RotationShimController` đang bọc ngoài RPP (dòng 98-99), nhưng `use_rotate_to_heading: false` (dòng 131).
- RPP không thể xoay tại chỗ khi gần goal → bị lố qua → planner tính lại → lặp xoắn ốc.
- **Giải pháp:** Bỏ `RotationShimController`, bật `use_rotate_to_heading: true` trên RPP. RPP sẽ tự dừng → xoay heading → dừng hẳn khi gần goal.

### Vấn đề 2: Không đi ngang (`vy = 0`)
- **Nguyên nhân gốc:** RPP **không hỗ trợ `vy`** ở mức thuật toán. Đây là giới hạn cứng, không thể config.
- **Giải pháp:** Không cần sửa RPP. Thay vào đó, dùng `RotationShimController` + `rotate_to_goal_heading: true` để **xoay đầu xe** trước khi di chuyển. Xe Mecanum xoay tại chỗ rất nhanh (0 bán kính), nên hiệu quả tương đương đi ngang.

> [!IMPORTANT]
> RPP **không bao giờ có thể đi ngang thật sự** (strafing). Nếu bạn cần xe đi ngang qua khe hẹp mà không xoay đầu, **bắt buộc phải dùng MPPI hoặc DWB**. Option B chỉ giúp RPP hoạt động **ổn định** trên Mecanum, không phải phát huy tối đa khả năng Mecanum.

---

## 2. Proposed Changes

### 2.1 Controller Config — `param_mini_mec.yaml`

#### [MODIFY] [param_mini_mec.yaml](file:///d:/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_mini_mec.yaml)

**Phương án A: RPP thuần (Bỏ RotationShimController)**
```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  # Không còn RotationShimController bọc ngoài
  
  desired_linear_vel: 0.3         # Giảm từ 0.4 → 0.3 (ổn định hơn)
  lookahead_dist: 0.6
  min_lookahead_dist: 0.3
  max_lookahead_dist: 0.9
  lookahead_time: 1.5
  
  use_velocity_scaled_lookahead_dist: true
  min_approach_linear_velocity: 0.05
  approach_velocity_scaling_dist: 0.6
  
  use_regulated_linear_velocity_scaling: true
  use_cost_regulated_linear_velocity_scaling: false
  regulated_linear_scaling_min_radius: 0.9
  regulated_linear_scaling_min_speed: 0.1
  
  min_turning_radius: 0.0          # Mecanum có thể xoay tại chỗ
  
  # ─── KEY FIX: Bật xoay tại chỗ ───
  use_rotate_to_heading: true      # ← FIX: Robot dừng + xoay heading thay vì cua vòng
  rotate_to_heading_min_angle: 0.785  # Xoay khi lệch > 45°
  rotate_to_heading_angular_vel: 1.0  # Tốc độ xoay (rad/s)
  max_angular_accel: 2.0
  
  transform_tolerance: 0.1
  max_robot_pose_search_dist: 10.0
  use_interpolation: true
```

**Phương án B: RPP + RotationShimController (Giữ Shim, sửa config)**
```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  angular_dist_threshold: 0.785     # Xoay khi lệch > 45°
  forward_sampling_distance: 0.5
  rotate_to_heading_angular_vel: 1.0
  max_angular_accel: 2.0
  simulate_ahead_time: 1.0
  rotate_to_goal_heading: true      # ← Xoay heading khi đến goal

  # RPP bên trong
  desired_linear_vel: 0.3
  lookahead_dist: 0.6
  min_lookahead_dist: 0.3
  max_lookahead_dist: 0.9
  lookahead_time: 1.5
  
  use_velocity_scaled_lookahead_dist: true
  min_approach_linear_velocity: 0.05
  approach_velocity_scaling_dist: 0.6
  
  use_regulated_linear_velocity_scaling: true
  use_cost_regulated_linear_velocity_scaling: false
  regulated_linear_scaling_min_radius: 0.9
  regulated_linear_scaling_min_speed: 0.1
  
  min_turning_radius: 0.0
  
  use_rotate_to_heading: false      # false vì Shim đang xử lý
  transform_tolerance: 0.1
  max_robot_pose_search_dist: 10.0
  use_interpolation: true
```

> [!WARNING]
> **Phương án A** vs **Phương án B:**
> - **A (RPP thuần):** Đơn giản, ít lỗi. Nhưng robot phải dừng + xoay mỗi khi lệch heading > 45° (hơi gián đoạn).
> - **B (Shim + RPP):** Mượt hơn. Shim xoay đầu trước khi chạy VÀ xoay heading khi đến goal. Nhưng cần đảm bảo `rotate_to_goal_heading: true` hoạt động đúng.
> - **Khuyến nghị: Phương án A** — đơn giản, ít rủi ro, dễ debug.

---

### 2.2 Approach Velocity — Giảm tốc mượt khi gần goal

Tham số quan trọng nhất để tránh orbiting:

```yaml
# Hiện tại
min_approach_linear_velocity: 0.05    # Robot giảm tốc xuống 0.05 m/s khi gần goal
approach_velocity_scaling_dist: 0.6   # Bắt đầu giảm tốc cách goal 0.6m

# Đề xuất giữ nguyên — giá trị hiện tại đã hợp lý
```

### 2.3 Goal Tolerance — Nới lỏng nếu cần

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.25    # Chấp nhận sai số 25cm (giữ nguyên)
  yaw_goal_tolerance: 0.25   # Chấp nhận sai số heading 14° (giữ nguyên)
```

Nếu robot vẫn xoay vòng sau khi áp dụng Phương án A, nới thêm:
```yaml
  xy_goal_tolerance: 0.30    # Nới lên 30cm
  yaw_goal_tolerance: 0.35   # Nới lên 20°
```

---

## 3. Kế hoạch thực hiện

### Phase 1: Sửa Config RPP (5 phút)
- [ ] Chọn Phương án A hoặc B
- [ ] Sửa `param_mini_mec.yaml` — phần `FollowPath`
- [ ] Giảm `desired_linear_vel: 0.4 → 0.3`

### Phase 2: Build & Test (10 phút)
- [ ] `colcon build --packages-select wheeltec_nav2` trên RPi
- [ ] `ros2 launch turn_on_wheeltec_robot prod_bringup.launch.py`
- [ ] Test case 1: Đặt goal gần (1m) → kiểm tra không orbiting
- [ ] Test case 2: Đặt goal xa (5m) → kiểm tra bám đường chuẩn
- [ ] Test case 3: Đặt goal yêu cầu xoay heading 90° → kiểm tra xoay tại chỗ

### Phase 3: Fine-tune (nếu cần)
- [ ] Nếu vẫn orbiting → nới `yaw_goal_tolerance: 0.35`
- [ ] Nếu xoay quá chậm → tăng `rotate_to_heading_angular_vel: 1.5`
- [ ] Nếu xoay quá nhanh → giảm `max_angular_accel: 1.5`

---

## 4. So sánh trước/sau

| Hành vi | Trước (RPP + Shim, rotate_to_heading: false) | Sau (RPP thuần, rotate_to_heading: true) |
|---------|-----------------------------------------------|------------------------------------------|
| Bám đường | ✅ Chuẩn | ✅ Chuẩn (giữ nguyên) |
| Gần goal | ❌ Xoay vòng | ✅ Dừng → xoay heading → dừng |
| Lệch heading | Shim xoay trước khi chạy | RPP tự dừng + xoay |
| Đi ngang (`vy`) | ❌ Không có | ❌ Vẫn không có (giới hạn RPP) |
| CPU | ~8% | ~5% (bỏ Shim, nhẹ hơn) |

---

## 5. Lưu ý quan trọng

> [!CAUTION]
> Nếu sau khi test Phase 2, bạn thấy robot vẫn không đủ linh hoạt (ví dụ: không lách qua khe cửa hẹp được vì phải xoay đầu 90°), thì **RPP không phải thuật toán phù hợp cho Mecanum**. Lúc đó quay lại **MPPI** (Option C) là quyết định đúng đắn.

> [!TIP]
> RPP phù hợp nhất cho: hành lang rộng, đường thẳng, ít chướng ngại vật di động, không cần đi ngang.
> MPPI phù hợp nhất cho: không gian phức tạp, cần strafing, nhiều chướng ngại vật động, tận dụng hết Mecanum.
