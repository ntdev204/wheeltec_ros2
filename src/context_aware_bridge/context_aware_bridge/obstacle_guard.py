"""
ObstacleGuard — Radial Safety Guard
====================================
Cơ chế an toàn mạnh nhất trong pipeline điều khiển.

Nguyên lý:
  - Mỗi vòng lặp, chiếu vector vận tốc robot (vx, vy) thành một hướng di chuyển.
  - Quét toàn bộ tia Lidar trong hình quạt ±half_cone về phía đó.
  - Nếu BẤT KỲ tia nào < stop_radius → VÔ HIỆU HÓA HOÀN TOÀN component đó (hard veto, không scale).
  - Lidar lệch 10cm về phía trước robot → tất cả tia được bù offset trước khi so sánh.

Chế độ:
  - Normal:          stop_radius = 0.50m, slow_radius = 0.70m
  - Narrow corridor: stop_radius = 0.30m, slow_radius = 0.50m (toggle qua set_narrow_mode)

Tolerance: ±5cm (STOP_TOLERANCE) — robot DỪNG ngay khi khoảng cách ≤ stop_radius + 5cm.
"""
from __future__ import annotations

import math
from typing import Optional

# Bù offset Lidar: Lidar lắp lệch 10cm về phía trước robot.
# Tia Lidar đo từ điểm gắn lidar → phải trừ đi offset này để về tâm robot.
LIDAR_FORWARD_OFFSET_M = 0.10  # m

# Góc quạt kiểm tra mỗi hướng (half-cone). 60° total → ±30° cho mỗi component.
HALF_CONE_DEG = 30.0

# Tolerance: dừng sớm thêm 5cm so với ngưỡng stop.
STOP_TOLERANCE_M = 0.05

# Ngưỡng vận tốc tối thiểu để coi là "đang di chuyển".
VEL_THRESHOLD = 0.02

# Ngưỡng giảm tốc mềm (linear scale từ slow_radius → stop_radius)
# Chỉ áp dụng khi không trong vùng hard-stop.
SLOW_SCALE_ENABLED = True


class ObstacleGuard:
    """
    Radial-based obstacle guard. Nhận scan360 degrees thay vì sectors 4 hướng.

    Args:
        stop_radius:       Bán kính dừng khẩn cấp (hard veto). Default 0.50m.
        slow_radius:       Bán kính bắt đầu giảm tốc. Default 0.70m.
        narrow_stop_radius: Bán kính dừng khi trong hành lang hẹp. Default 0.30m.
        narrow_slow_radius: Bán kính giảm tốc khi hẹp. Default 0.50m.
        scan_stale_timeout: Nếu scan cũ hơn giá trị này (giây) → dừng toàn bộ. Default 0.75s.
        lidar_offset_m:    Offset Lidar về phía trước tâm robot (m). Default 0.10m.
    """

    def __init__(
        self,
        stop_radius: float = 0.50,
        slow_radius: float = 0.70,
        narrow_stop_radius: float = 0.30,
        narrow_slow_radius: float = 0.50,
        scan_stale_timeout: float = 0.75,
        lidar_offset_m: float = LIDAR_FORWARD_OFFSET_M,
    ):
        self._normal_stop  = float(stop_radius)
        self._normal_slow  = float(slow_radius)
        self._narrow_stop  = float(narrow_stop_radius)
        self._narrow_slow  = float(narrow_slow_radius)
        self.scan_stale_timeout = float(scan_stale_timeout)
        self._lidar_offset = float(lidar_offset_m)
        self._narrow_mode  = False

        # Cache bán kính hiện tại (cập nhật bởi set_narrow_mode)
        self._stop_radius = self._normal_stop
        self._slow_radius = self._normal_slow

    # ─── Public API ──────────────────────────────────────────────────────────

    def set_narrow_mode(self, enabled: bool) -> None:
        """Chuyển sang chế độ hành lang hẹp (stop=30cm) hoặc normal (stop=50cm)."""
        self._narrow_mode = enabled
        if enabled:
            self._stop_radius = self._narrow_stop
            self._slow_radius = self._narrow_slow
        else:
            self._stop_radius = self._normal_stop
            self._slow_radius = self._normal_slow

    def guard(self, twist, scan360: Optional[list], scan_age_s: float):
        """
        Lọc lệnh điều khiển dựa trên Lidar 360°.

        Args:
            twist:       Đối tượng có .linear.x, .linear.y, .angular.z
            scan360:     List 360 phần tử, index = degree (0=front, 90=left ROS frame),
                         giá trị = khoảng cách tối thiểu trong bin đó (m).
                         None hoặc rỗng = fail-safe dừng toàn bộ.
            scan_age_s:  Tuổi của scan tính bằng giây.

        Returns:
            twist đã được filter (in-place).
        """
        # Fail-safe: Scan quá cũ hoặc không có dữ liệu → dừng toàn bộ
        if scan_age_s > self.scan_stale_timeout or not scan360:
            self._zero(twist)
            return twist

        vx = twist.linear.x
        vy = twist.linear.y

        # ── Kiểm tra trục X (tiến/lùi) ──────────────────────────────────────
        if abs(vx) > VEL_THRESHOLD:
            # Tiến (vx > 0) → hướng 0° (front), Lùi (vx < 0) → hướng 180° (rear)
            direction_deg = 0.0 if vx > 0 else 180.0
            min_dist = self._min_in_cone(scan360, direction_deg, HALF_CONE_DEG, forward=(vx > 0))
            twist.linear.x = self._apply_guard(vx, min_dist)

        # ── Kiểm tra trục Y (trái/phải mecanum) ──────────────────────────────
        if abs(vy) > VEL_THRESHOLD:
            # Stafe left (vy > 0) → 90° (left), Strafe right (vy < 0) → 270° (right)
            direction_deg = 90.0 if vy > 0 else 270.0
            min_dist = self._min_in_cone(scan360, direction_deg, HALF_CONE_DEG, forward=False)
            twist.linear.y = self._apply_guard(vy, min_dist)

        # ── Nếu cả 2 trục bị cắt thì angular cũng dừng ─────────────────────
        if abs(twist.linear.x) < 1e-4:
            twist.linear.x = 0.0
        if abs(twist.linear.y) < 1e-4:
            twist.linear.y = 0.0
        if twist.linear.x == 0.0 and twist.linear.y == 0.0:
            twist.angular.z = 0.0

        return twist

    # ─── Private helpers ──────────────────────────────────────────────────────

    def _min_in_cone(
        self,
        scan360: list,
        center_deg: float,
        half_cone_deg: float,
        forward: bool,
    ) -> float:
        """
        Tìm khoảng cách tối thiểu trong hình quạt [center ± half_cone].
        Bù offset Lidar: nếu Lidar lệch về phía trước 10cm,
          - Khi kiểm tra hướng FORWARD (tia đang đi về phía trước lidar): khoảng
            cách thực = scan_dist - lidar_offset (lidar đã gần hơn tâm robot).
          - Khi kiểm tra hướng BACKWARD hoặc SIDE: khoảng cách thực = scan_dist + lidar_offset
            (lidar xa hơn tâm robot về phía đó).
        """
        n = len(scan360)
        start = center_deg - half_cone_deg
        end   = center_deg + half_cone_deg
        min_dist = float('inf')

        angle = start
        while angle <= end:
            idx = int(round(angle)) % 360
            raw = scan360[idx]
            if math.isfinite(raw) and raw > 0:
                # Bù offset Lidar
                if forward:
                    corrected = raw - self._lidar_offset  # Lidar gần vật hơn tâm
                else:
                    corrected = raw + self._lidar_offset  # Lidar xa vật hơn tâm
                corrected = max(0.0, corrected)
                min_dist = min(min_dist, corrected)
            angle += 1.0  # bước 1 degree

        return min_dist if min_dist < float('inf') else 9.9

    def _apply_guard(self, vel_component: float, min_dist: float) -> float:
        """
        Hard veto: khoảng cách ≤ stop_radius + tolerance → về 0 hoàn toàn.
        Soft scale: giữa stop_radius và slow_radius → giảm tuyến tính.
        """
        hard_stop = self._stop_radius + STOP_TOLERANCE_M
        if min_dist <= hard_stop:
            return 0.0  # HARD VETO — mạnh nhất
        if SLOW_SCALE_ENABLED and min_dist < self._slow_radius:
            span = max(self._slow_radius - hard_stop, 1e-6)
            scale = (min_dist - hard_stop) / span
            scale = max(0.0, min(1.0, scale))
            return vel_component * scale
        return vel_component

    @staticmethod
    def _zero(twist) -> None:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
