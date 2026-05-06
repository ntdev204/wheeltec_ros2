"""
ObstacleGuard — Magnetic Repulsion Safety Guard
================================================
Mô hình: robot và vật cản là nam châm CÙNG CỰC → đẩy nhau.

Hành vi:
  - v_final = v_commanded + v_repulsion
  - v_repulsion: hướng RA XA vật cản, tối đa 0.2 m/s
  - Nếu v_final vẫn đi vào vùng stop_radius → zero velocity

Vùng khoảng cách (từ TÂM robot, đã bù offset lidar 10cm):
  > influence_radius (1.0m) : Tự do, không có lực đẩy
  slow_radius → influence_r : Bắt đầu bị đẩy ngược (tăng dần → max 0.2 m/s)
  ≤ stop_radius + 5cm       : Velocity = 0 (vi phạm)

Stateless: xử lý inline mỗi lần gọi, không lưu state.
"""
from __future__ import annotations

import math
from typing import Optional

LIDAR_FORWARD_OFFSET_M = 0.10   # m
HALF_CONE_DEG          = 30     # ±30° kiểm tra mỗi hướng
STOP_TOLERANCE_M       = 0.05   # buffer an toàn thêm 5cm
VEL_THRESHOLD          = 0.02   # m/s

REPULSION_MAX_SPEED    = 0.20   # m/s — tốc độ đẩy tối đa


class ObstacleGuard:
    """
    Magnetic repulsion guard.

    guard(twist, scan360, scan_age_s):
        scan360: list 360 phần tử, index = degree (0=trước,90=trái,180=sau,270=phải)
    """

    def __init__(
        self,
        stop_radius:        float = 0.50,
        slow_radius:        float = 0.70,
        narrow_stop_radius: float = 0.30,
        narrow_slow_radius: float = 0.50,
        influence_radius:   float = 1.00,
        scan_stale_timeout: float = 0.75,
        lidar_offset_m:     float = LIDAR_FORWARD_OFFSET_M,
    ):
        self._normal_stop  = float(stop_radius)
        self._normal_slow  = float(slow_radius)
        self._narrow_stop  = float(narrow_stop_radius)
        self._narrow_slow  = float(narrow_slow_radius)
        self._influence_r  = float(influence_radius)
        self.scan_stale_timeout = float(scan_stale_timeout)
        self._lidar_offset = float(lidar_offset_m)

        self._stop_radius = self._normal_stop
        self._slow_radius = self._normal_slow

    # ─── Public API ───────────────────────────────────────────────────────────

    def set_narrow_mode(self, enabled: bool) -> None:
        """Hành lang hẹp: stop=30cm, slow=50cm."""
        if enabled:
            self._stop_radius, self._slow_radius = self._narrow_stop, self._narrow_slow
        else:
            self._stop_radius, self._slow_radius = self._normal_stop, self._normal_slow

    def guard(self, twist, scan360: Optional[list], scan_age_s: float):
        """
        Áp dụng lực đẩy nam châm lên lệnh điều khiển.

        v_final = v_commanded + v_repulsion (max 0.2 m/s)
        Nếu v_final đi vào stop_radius → zero.
        """
        if scan_age_s > self.scan_stale_timeout or not scan360:
            self._zero(twist)
            return twist

        cmd_x = float(twist.linear.x)
        cmd_y = float(twist.linear.y)

        # ── Bước 1: Tính lực đẩy nam châm từ toàn bộ 360° ───────────────────
        rep_vx, rep_vy = self._magnetic_repulsion(scan360)

        # ── Bước 2: Hợp lực: lệnh người dùng + lực đẩy ──────────────────────
        vx = cmd_x + rep_vx
        vy = cmd_y + rep_vy

        # ── Bước 3: Hard stop — kiểm tra cả hướng v_final (cmd + repulsion) ───
        # Dùng v_final direction, không phải v_commanded, để ngăn repulsion
        # đẩy robot vào vùng đã bị block ở phía đối diện.
        hard_stop = self._stop_radius + STOP_TOLERANCE_M

        if abs(cmd_x) > VEL_THRESHOLD:
            center = 0 if cmd_x > 0 else 180
            if self._min_eff_in_cone(scan360, center) <= hard_stop:
                vx = 0.0

        if abs(cmd_y) > VEL_THRESHOLD:
            center = 90 if cmd_y > 0 else 270
            if self._min_eff_in_cone(scan360, center) <= hard_stop:
                vy = 0.0

        # Không tự tạo chuyển động khi không có lệnh teleop trên trục đó.
        # Tránh hiện tượng đâm tường xong robot tự lùi/chạy ngang.
        if abs(cmd_x) <= VEL_THRESHOLD:
            vx = 0.0
        if abs(cmd_y) <= VEL_THRESHOLD:
            vy = 0.0

        # Bổ sung: nếu repulsion muốn đẩy robot vào hướng đã bị block → zero rep đó
        # (ví dụ: tường sau đẩy tiến nhưng phía trước đã trong stop zone)
        if abs(rep_vx) > VEL_THRESHOLD:
            rep_center = 0 if rep_vx > 0 else 180
            if self._min_eff_in_cone(scan360, rep_center) <= hard_stop:
                vx = twist.linear.x  # revert về v_commanded (không có repulsion)
                if abs(vx) > VEL_THRESHOLD:
                    center = 0 if vx > 0 else 180
                    if self._min_eff_in_cone(scan360, center) <= hard_stop:
                        vx = 0.0

        if abs(rep_vy) > VEL_THRESHOLD:
            rep_center = 90 if rep_vy > 0 else 270
            if self._min_eff_in_cone(scan360, rep_center) <= hard_stop:
                vy = twist.linear.y
                if abs(vy) > VEL_THRESHOLD:
                    center = 90 if vy > 0 else 270
                    if self._min_eff_in_cone(scan360, center) <= hard_stop:
                        vy = 0.0

        # ── Bước 4: Ghi lại ──────────────────────────────────────────────────
        twist.linear.x = 0.0 if abs(vx) < 1e-4 else vx
        twist.linear.y = 0.0 if abs(vy) < 1e-4 else vy
        if twist.linear.x == 0.0 and twist.linear.y == 0.0:
            twist.angular.z = 0.0

        return twist

    # ─── Private ─────────────────────────────────────────────────────────────

    def _magnetic_repulsion(self, scan360: list) -> tuple[float, float]:
        """
        Tính vector lực đẩy từ TẤT CẢ vật cản trong influence_radius.

        Mỗi vật cản tại góc `deg` đóng góp lực đẩy NGƯỢC CHIỀU (deg+180°),
        trọng số tăng theo bình phương khi gần hơn.
        Vector tổng được chuẩn hoá và scale vào [0, REPULSION_MAX_SPEED].
        """
        fx, fy = 0.0, 0.0
        for deg in range(360):
            raw = scan360[deg] if deg < len(scan360) else 9.9
            eff = self._eff_dist(raw, deg)
            if 0 < eff < self._influence_r:
                # Trọng số: 0 tại edge, tăng → 1 khi sát vật cản
                weight = ((self._influence_r - eff) / self._influence_r) ** 2
                deg_rad = math.radians(deg)
                # Hướng đẩy = ngược hướng vật cản
                fx -= weight * math.cos(deg_rad)
                fy -= weight * math.sin(deg_rad)

        # Chuẩn hoá rồi scale về [0, REPULSION_MAX_SPEED]
        mag = math.sqrt(fx * fx + fy * fy)
        if mag > 1e-6:
            # Nếu tổng trọng số > 1 → chuẩn hoá trước
            if mag > 1.0:
                fx /= mag
                fy /= mag
            fx *= REPULSION_MAX_SPEED
            fy *= REPULSION_MAX_SPEED

        return fx, fy

    def _min_eff_in_cone(self, scan360: list, center_deg: int) -> float:
        """Khoảng cách hiệu dụng nhỏ nhất trong hình quạt ±30° quanh center_deg."""
        min_eff = 9.9
        for offset in range(-HALF_CONE_DEG, HALF_CONE_DEG + 1):
            deg = (center_deg + offset) % 360
            raw = scan360[deg] if deg < len(scan360) else 9.9
            eff = self._eff_dist(raw, deg)
            if eff < min_eff:
                min_eff = eff
        return min_eff

    def _eff_dist(self, raw: float, deg: int) -> float:
        """Khoảng cách hiệu dụng từ tâm robot (bù offset lidar 10cm)."""
        if not math.isfinite(raw) or raw <= 0:
            return 9.9
        return max(0.0, raw - self._lidar_offset * math.cos(math.radians(deg)))

    @staticmethod
    def _zero(twist) -> None:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
