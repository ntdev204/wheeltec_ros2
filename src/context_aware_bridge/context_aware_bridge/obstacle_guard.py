"""
ObstacleGuard — Virtual Force Field (VFF) Safety Guard
======================================================
Cơ chế: Khi robot tiến về hướng nào và gặp vật cản, thay vì dừng cứng,
hệ thống tính lực đẩy tổng hợp từ MỌI vật cản xung quanh, chiếu lên
trục vuông góc với hướng di chuyển → tự động thêm angular.z để né.

Hoạt động cho MỌI hướng:
  - Tiến (vx>0), vật cản nghiêng phải → tăng angular.z (né trái)
  - Tiến (vx>0), vật cản nghiêng trái → giảm angular.z (né phải)
  - Sang phải (vy<0), vật cản phía dưới-phải → thêm angular.z tương ứng
  - Lùi, strafe — tương tự, tất cả dựa trên lực đẩy tổng hợp 360°

3 vùng khoảng cách:
  ┌─────────────────────────────────────────────────┐
  │ > influence_radius (1.0m)  : Tự do             │
  │ slow_radius → influence_r  : Bắt đầu tính VFF  │
  │ stop_radius → slow_radius  : Giảm tốc + VFF    │
  │ ≤ stop_radius + 5cm        : DỪNG + VFF angular │
  └─────────────────────────────────────────────────┘

Lidar offset 10cm về phía trước: bù bằng cosine projection.
Stateless: không lưu state, xử lý inline mỗi lệnh.
"""
from __future__ import annotations

import math
from typing import Optional

LIDAR_FORWARD_OFFSET_M = 0.10   # m — lidar lệch về phía trước so với tâm robot
HALF_CONE_DEG          = 30     # ±30° = hình quạt 60° kiểm tra mỗi hướng
STOP_TOLERANCE_M       = 0.05   # dừng sớm thêm 5cm (buffer an toàn)
VEL_THRESHOLD          = 0.02   # m/s — ngưỡng nhận biết "đang di chuyển"

# VFF parameters
VFF_ANGULAR_GAIN       = 1.2    # hệ số khuếch đại lực đẩy → angular.z
VFF_MAX_ANGULAR_CORR   = 1.5    # rad/s — giới hạn tối đa angular correction
VFF_INFLUENCE_RADIUS   = 1.0    # m — bán kính tính lực đẩy VFF


class ObstacleGuard:
    """
    VFF-based obstacle guard. Né vật cản bằng angular steering, không chỉ dừng.

    Stateless — không lưu state giữa các lần gọi.
    """

    def __init__(
        self,
        stop_radius:         float = 0.50,
        slow_radius:         float = 0.70,
        narrow_stop_radius:  float = 0.30,
        narrow_slow_radius:  float = 0.50,
        influence_radius:    float = VFF_INFLUENCE_RADIUS,
        angular_gain:        float = VFF_ANGULAR_GAIN,
        max_angular_corr:    float = VFF_MAX_ANGULAR_CORR,
        scan_stale_timeout:  float = 0.75,
        lidar_offset_m:      float = LIDAR_FORWARD_OFFSET_M,
    ):
        self._normal_stop     = float(stop_radius)
        self._normal_slow     = float(slow_radius)
        self._narrow_stop     = float(narrow_stop_radius)
        self._narrow_slow     = float(narrow_slow_radius)
        self._influence_r     = float(influence_radius)
        self._angular_gain    = float(angular_gain)
        self._max_angular     = float(max_angular_corr)
        self.scan_stale_timeout = float(scan_stale_timeout)
        self._lidar_offset    = float(lidar_offset_m)

        self._stop_radius = self._normal_stop
        self._slow_radius = self._normal_slow

    # ─── Public API ───────────────────────────────────────────────────────────

    def set_narrow_mode(self, enabled: bool) -> None:
        """Hành lang hẹp: stop=30cm, slow=50cm. Normal: stop=50cm, slow=70cm."""
        if enabled:
            self._stop_radius, self._slow_radius = self._narrow_stop, self._narrow_slow
        else:
            self._stop_radius, self._slow_radius = self._normal_stop, self._normal_slow

    def guard(
        self,
        twist,
        scan360: Optional[list],
        scan_age_s: float,
    ):
        """
        Filter + VFF steering cho lệnh điều khiển.

        Khi vật cản xuất hiện trong hướng di chuyển:
          - Giảm tốc độ linear tuyến tính theo khoảng cách
          - Thêm angular.z dựa trên tổng lực đẩy VFF từ 360° scan
          - Chỉ zero linear khi quá gần (≤ stop_radius + 5cm)

        Args:
            twist:       Đối tượng có .linear.x, .linear.y, .angular.z
            scan360:     360 phần tử khoảng cách (m), index=degree (0=trước,90=trái)
            scan_age_s:  Tuổi scan (giây). Vượt timeout → dừng khẩn cấp.
        """
        # Fail-safe: scan cũ hoặc mất dữ liệu → dừng toàn bộ
        if scan_age_s > self.scan_stale_timeout or not scan360:
            self._zero(twist)
            return twist

        vx = twist.linear.x
        vy = twist.linear.y

        # ── Bước 1: Tính lực đẩy VFF tổng hợp từ toàn bộ 360° ───────────────
        # rep_x, rep_y: vector lực đẩy trong robot frame (x=trước, y=trái)
        rep_x, rep_y = self._repulsive_force(scan360)

        # ── Bước 2: Scale velocity theo khoảng cách vật cản phía trước/sau ──
        if abs(vx) > VEL_THRESHOLD:
            center = 0 if vx > 0 else 180
            min_eff = self._min_eff_in_cone(scan360, center)
            vx = self._scale_velocity(vx, min_eff)

        if abs(vy) > VEL_THRESHOLD:
            center = 90 if vy > 0 else 270
            min_eff = self._min_eff_in_cone(scan360, center)
            vy = self._scale_velocity(vy, min_eff)

        # ── Bước 3: VFF angular steering ─────────────────────────────────────
        # Chiếu lực đẩy lên trục vuông góc với hướng di chuyển.
        # Cross product (motion × repulsive) → angular correction.
        motion_mag = math.sqrt(vx * vx + vy * vy)
        if motion_mag > VEL_THRESHOLD and (abs(rep_x) > 1e-4 or abs(rep_y) > 1e-4):
            # Normalise hướng di chuyển
            vx_n = vx / motion_mag
            vy_n = vy / motion_mag
            # z-component của cross product: steer_z = vx_n*rep_y - vy_n*rep_x
            # Dương → steer trái (angular.z tăng), âm → steer phải
            steer = (vx_n * rep_y - vy_n * rep_x) * self._angular_gain
            steer = max(-self._max_angular, min(self._max_angular, steer))
            twist.angular.z += steer

        # ── Bước 4: Ghi lại velocity đã filter ────────────────────────────────
        twist.linear.x = vx
        twist.linear.y = vy

        # Dọn giá trị cực nhỏ
        if abs(twist.linear.x) < 1e-4: twist.linear.x = 0.0
        if abs(twist.linear.y) < 1e-4: twist.linear.y = 0.0
        # Nếu cả hai linear về 0 → angular cũng về 0
        if twist.linear.x == 0.0 and twist.linear.y == 0.0:
            twist.angular.z = 0.0

        return twist

    # ─── Private ─────────────────────────────────────────────────────────────

    def _repulsive_force(self, scan360: list) -> tuple[float, float]:
        """
        Tính tổng lực đẩy từ TẤT CẢ vật cản trong vòng influence_radius.

        Mỗi vật cản tại góc deg đóng góp một vector lực NGƯỢC chiều với
        hướng từ robot → vật cản. Trọng số tăng khi vật cản gần hơn (quadratic).

        Returns:
            (fx, fy) trong robot frame: x=trước, y=trái
        """
        fx, fy = 0.0, 0.0
        for deg in range(360):
            raw = scan360[deg] if deg < len(scan360) else 9.9
            eff = self._eff_dist(raw, deg)
            if eff < self._influence_r:
                # Trọng số tăng theo bình phương khi vật cản gần hơn
                weight = ((self._influence_r - eff) / self._influence_r) ** 2
                deg_rad = math.radians(deg)
                # Hướng từ robot → vật cản: (cos, sin)
                # Lực đẩy ngược chiều: -(cos, sin)
                fx -= weight * math.cos(deg_rad)
                fy -= weight * math.sin(deg_rad)
        return fx, fy

    def _min_eff_in_cone(self, scan360: list, center_deg: int) -> float:
        """Khoảng cách hiệu dụng nhỏ nhất trong hình quạt ±HALF_CONE_DEG."""
        min_eff = 9.9
        for offset in range(-HALF_CONE_DEG, HALF_CONE_DEG + 1):
            deg = (center_deg + offset) % 360
            raw = scan360[deg] if deg < len(scan360) else 9.9
            eff = self._eff_dist(raw, deg)
            if eff < min_eff:
                min_eff = eff
        return min_eff

    def _scale_velocity(self, vel: float, min_eff_dist: float) -> float:
        """
        Scale velocity theo khoảng cách:
          ≤ stop+tol → 0.0 (dừng, để VFF angular lái)
          stop+tol → slow → ramp tuyến tính
          ≥ slow    → giữ nguyên
        """
        hard_stop = self._stop_radius + STOP_TOLERANCE_M
        if min_eff_dist <= hard_stop:
            return 0.0
        if min_eff_dist >= self._slow_radius:
            return vel
        span = max(self._slow_radius - hard_stop, 1e-6)
        scale = (min_eff_dist - hard_stop) / span
        return vel * max(0.0, min(1.0, scale))

    def _eff_dist(self, raw: float, deg: int) -> float:
        """
        Khoảng cách hiệu dụng từ TÂM ROBOT (bù offset lidar 10cm).
        eff = raw - lidar_offset * cos(deg_rad)
          deg=0 (trước): eff = raw - 10cm  (lidar gần vật hơn tâm)
          deg=180 (sau) : eff = raw + 10cm  (lidar xa vật hơn tâm)
          deg=90/270    : eff ≈ raw          (không ảnh hưởng)
        """
        if not math.isfinite(raw) or raw <= 0:
            return 9.9
        eff = raw - self._lidar_offset * math.cos(math.radians(deg))
        return max(0.0, eff)

    @staticmethod
    def _zero(twist) -> None:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
