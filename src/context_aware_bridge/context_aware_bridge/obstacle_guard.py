"""
ObstacleGuard — Stateless Radial Safety Guard
=============================================
Cơ chế an toàn MẠNH NHẤT trong pipeline điều khiển.

Thiết kế: STATELESS — không lưu bản đồ, không cache, không delay.
  - Nhận scan360 raw NGAY KHI có lệnh điều khiển.
  - scan360 từ callback Lidar ĐÃ là bản đồ 360° real-time, không cần copy thêm.
  - Tính toán inline, trả kết quả, không lưu state.

Nguyên lý hard-veto:
  - Chiếu vector vận tốc → xác định hướng đang muốn đi.
  - Quét hình quạt ±30° trong scan360 hiện tại về phía đó.
  - Nếu bất kỳ tia nào có effective_dist ≤ stop_radius + 5cm → ZERO component đó.
  - Áp dụng cho MỌI nguồn điều khiển: keyboard, AI, Nav2.

Lidar offset (10cm về phía trước):
  - Dùng projection cosine theo từng góc: eff = raw - offset * cos(deg)
  - Hướng trước (0°): eff = raw - 10cm (lidar gần hơn tâm robot)
  - Hướng sau (180°): eff = raw + 10cm (lidar xa hơn tâm robot)
  - Hướng bên (90°/270°): eff ≈ raw (không ảnh hưởng)
"""
from __future__ import annotations

import math
from typing import Optional

LIDAR_FORWARD_OFFSET_M = 0.10   # m — lidar lệch về phía trước so với tâm robot
HALF_CONE_DEG          = 30     # ±30° = hình quạt 60° mỗi hướng
STOP_TOLERANCE_M       = 0.05   # dừng sớm thêm 5cm (buffer an toàn)
VEL_THRESHOLD          = 0.02   # m/s — ngưỡng nhận biết "đang di chuyển"


class ObstacleGuard:
    """
    Stateless radial safety guard.

    Sử dụng:
        guard = ObstacleGuard(stop_radius=0.50, slow_radius=0.70)

        # Trong recv_loop khi có lệnh điều khiển:
        scan_age = time.monotonic() - self._last_scan_time
        twist = guard.guard(twist, scan360, scan_age)

    scan360: list[float] — 360 phần tử, index = degree trong robot frame
        (0°=trước, 90°=trái, 180°=sau, 270°=phải),
        giá trị = khoảng cách lidar đo được tại bin đó (m), 9.9 nếu không có dữ liệu.
    """

    def __init__(
        self,
        stop_radius:         float = 0.50,
        slow_radius:         float = 0.70,
        narrow_stop_radius:  float = 0.30,
        narrow_slow_radius:  float = 0.50,
        scan_stale_timeout:  float = 0.75,
        lidar_offset_m:      float = LIDAR_FORWARD_OFFSET_M,
    ):
        self._normal_stop = float(stop_radius)
        self._normal_slow = float(slow_radius)
        self._narrow_stop = float(narrow_stop_radius)
        self._narrow_slow = float(narrow_slow_radius)
        self.scan_stale_timeout = float(scan_stale_timeout)
        self._lidar_offset = float(lidar_offset_m)

        # Chế độ hiện tại
        self._stop_radius = self._normal_stop
        self._slow_radius = self._normal_slow

    # ─── Public API ───────────────────────────────────────────────────────────

    def set_narrow_mode(self, enabled: bool) -> None:
        """Chuyển sang chế độ hành lang hẹp (stop=30cm) hoặc bình thường (stop=50cm)."""
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
        Filter lệnh điều khiển dựa trên scan360 HIỆN TẠI.
        Stateless — không lưu bất kỳ state nào giữa các lần gọi.

        Args:
            twist:       Đối tượng có .linear.x, .linear.y, .angular.z
            scan360:     List 360 phần tử khoảng cách (m). None = fail-safe dừng.
            scan_age_s:  Tuổi scan (giây). > timeout → fail-safe dừng.

        Returns:
            twist đã được filter (in-place).
        """
        # Fail-safe: scan cũ hoặc không có → dừng toàn bộ ngay
        if scan_age_s > self.scan_stale_timeout or not scan360:
            self._zero(twist)
            return twist

        vx = twist.linear.x
        vy = twist.linear.y

        # ── Trục X: tiến (0°) / lùi (180°) ─────────────────────────────────
        if abs(vx) > VEL_THRESHOLD:
            center = 0 if vx > 0 else 180
            blocked, scale = self._check_cone(scan360, center)
            twist.linear.x = 0.0 if blocked else vx * scale

        # ── Trục Y: strafe trái (90°) / strafe phải (270°) ──────────────────
        if abs(vy) > VEL_THRESHOLD:
            center = 90 if vy > 0 else 270
            blocked, scale = self._check_cone(scan360, center)
            twist.linear.y = 0.0 if blocked else vy * scale

        # ── Nếu cả 2 trục bị veto → angular cũng về 0 ───────────────────────
        if abs(twist.linear.x) < 1e-4: twist.linear.x = 0.0
        if abs(twist.linear.y) < 1e-4: twist.linear.y = 0.0
        if twist.linear.x == 0.0 and twist.linear.y == 0.0:
            twist.angular.z = 0.0

        return twist

    # ─── Private ─────────────────────────────────────────────────────────────

    def _check_cone(self, scan360: list, center_deg: int) -> tuple[bool, float]:
        """
        Kiểm tra hình quạt ±HALF_CONE_DEG quanh center_deg TRONG scan360 hiện tại.
        Tính effective_dist inline (không cache).

        Returns:
            (is_blocked, min_scale_factor)
        """
        hard_stop = self._stop_radius + STOP_TOLERANCE_M
        min_scale = 1.0

        for offset in range(-HALF_CONE_DEG, HALF_CONE_DEG + 1):
            deg = (center_deg + offset) % 360
            raw = scan360[deg] if deg < len(scan360) else 9.9

            # Effective distance: bù offset lidar theo cosine projection
            eff = self._effective_dist(raw, deg)

            # Hard veto — ngay lập tức
            if eff <= hard_stop:
                return True, 0.0

            # Soft slow-down zone
            if eff < self._slow_radius:
                span = max(self._slow_radius - hard_stop, 1e-6)
                scale = max(0.0, min(1.0, (eff - hard_stop) / span))
                if scale < min_scale:
                    min_scale = scale

        return False, min_scale

    def _effective_dist(self, raw: float, deg: int) -> float:
        """
        Khoảng cách hiệu dụng từ tâm robot tại góc deg.
        Bù offset lidar 10cm về phía trước: eff = raw - offset * cos(deg_rad).
        """
        if not math.isfinite(raw) or raw <= 0:
            return 9.9
        deg_rad = math.radians(deg)
        eff = raw - self._lidar_offset * math.cos(deg_rad)
        return max(0.0, eff)

    @staticmethod
    def _zero(twist) -> None:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
