"""Tests for VFF-based ObstacleGuard."""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from context_aware_bridge.obstacle_guard import ObstacleGuard


def make_twist(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=x, y=y),
        angular=SimpleNamespace(z=z),
    )


def scan_clear(default=9.9):
    return [default] * 360


def scan_with(angle_deg: int, dist_m: float, default=9.9):
    s = scan_clear(default)
    s[angle_deg % 360] = dist_m
    return s


class TestObstacleGuard:
    def setup_method(self):
        self.g = ObstacleGuard(
            stop_radius=0.50,
            slow_radius=0.70,
            influence_radius=1.0,
            angular_gain=1.2,
            lidar_offset_m=0.10,
            scan_stale_timeout=0.75,
        )

    # ── Hard stop khi quá gần ─────────────────────────────────────────────────

    def test_forward_hard_stop(self):
        """Lidar 0.45m → eff = 0.35m < 0.55 → linear.x = 0."""
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.45), 0.1)
        assert t.linear.x == 0.0

    def test_forward_pass_far(self):
        """Lidar 0.90m → eff = 0.80m > 0.55 → không dừng."""
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.90), 0.1)
        assert t.linear.x > 0.0

    def test_rear_hard_stop(self):
        """Lùi, lidar 0.40m phía sau → eff = 0.50m ≤ 0.55 → dừng."""
        t = self.g.guard(make_twist(x=-0.5), scan_with(180, 0.40), 0.1)
        assert t.linear.x == 0.0

    def test_strafe_left_hard_stop(self):
        t = self.g.guard(make_twist(y=0.5), scan_with(90, 0.45), 0.1)
        assert t.linear.y == 0.0

    def test_strafe_right_hard_stop(self):
        t = self.g.guard(make_twist(y=-0.5), scan_with(270, 0.45), 0.1)
        assert t.linear.y == 0.0

    # ── VFF angular steering ──────────────────────────────────────────────────

    def test_forward_obstacle_right_steers_left(self):
        """
        Tiến, vật cản bên phải (270° = right) trong vùng influence (0.8m).
        VFF đẩy sang trái → angular.z > 0 (ROS: CCW = trái).
        """
        scan = scan_clear()
        scan[270] = 0.8  # vật cản bên phải, vừa trong influence_radius
        t = self.g.guard(make_twist(x=0.5), scan, 0.1)
        # Vật cản phải → robot nên lái trái
        assert t.angular.z > 0.0, f"Expected steer left, got angular.z={t.angular.z}"

    def test_forward_obstacle_left_steers_right(self):
        """Tiến, vật cản bên trái (90°) → angular.z < 0 (lái phải)."""
        scan = scan_clear()
        scan[90] = 0.8
        t = self.g.guard(make_twist(x=0.5), scan, 0.1)
        assert t.angular.z < 0.0, f"Expected steer right, got angular.z={t.angular.z}"

    def test_forward_symmetric_obstacles_no_steer(self):
        """Vật cản đều hai bên → lực đẩy triệt tiêu → angular.z ≈ 0."""
        scan = scan_clear()
        scan[90]  = 0.8  # trái
        scan[270] = 0.8  # phải, cùng khoảng cách
        t = self.g.guard(make_twist(x=0.5), scan, 0.1)
        assert abs(t.angular.z) < 0.05, f"Expected ~0 steer, got {t.angular.z}"

    def test_obstacle_behind_does_not_affect_forward_angular(self):
        """Vật cản phía sau không tạo angular correction khi đang tiến."""
        scan = scan_clear()
        scan[180] = 0.5  # phía sau, gần
        t = self.g.guard(make_twist(x=0.5), scan, 0.1)
        # Lực đẩy từ phía sau → đẩy về phía trước → cross product với forward ≈ 0
        assert abs(t.angular.z) < 0.1

    def test_angular_correction_capped_at_max(self):
        """Angular correction không vượt quá max_angular_corr (1.5 rad/s)."""
        # Nhiều vật cản gần → lực đẩy lớn nhưng bị cap
        scan = scan_clear()
        for deg in range(225, 315):  # vật cản phải + phía sau-phải
            scan[deg] = 0.3
        t = self.g.guard(make_twist(x=0.5), scan, 0.1)
        assert abs(t.angular.z) <= 1.5 + 0.001

    # ── Soft slow-down ─────────────────────────────────────────────────────────

    def test_soft_slowdown(self):
        """Giữa stop và slow → giảm tốc tuyến tính."""
        # eff = 0.72 - 0.10 = 0.62; 0.55 < 0.62 < 0.70 → scale
        t = self.g.guard(make_twist(x=0.6), scan_with(0, 0.72), 0.1)
        assert 0.0 < t.linear.x < 0.6

    # ── Fail-safe ──────────────────────────────────────────────────────────────

    def test_stale_scan_stops_all(self):
        t = self.g.guard(make_twist(x=0.5, y=0.3, z=0.8), scan_clear(), 2.0)
        assert t.linear.x == 0.0
        assert t.linear.y == 0.0
        assert t.angular.z == 0.0

    def test_null_scan_stops_all(self):
        t = self.g.guard(make_twist(x=0.5), None, 0.1)
        assert t.linear.x == 0.0

    # ── Angular zeroed khi cả 2 linear bị stop ────────────────────────────────

    def test_angular_zeroed_when_fully_blocked(self):
        """Nếu vx và vy đều bị hard-stop, angular.z cũng về 0."""
        scan = scan_with(0, 0.20)
        scan[270] = 0.20
        t = self.g.guard(make_twist(x=0.5, y=-0.3, z=0.5), scan, 0.1)
        assert t.linear.x == 0.0
        assert t.linear.y == 0.0
        assert t.angular.z == 0.0

    # ── Narrow mode ────────────────────────────────────────────────────────────

    def test_narrow_mode_tighter(self):
        """Narrow: stop=0.30m. Lidar 0.37m → eff = 0.27m < 0.35m → DỪNG."""
        self.g.set_narrow_mode(True)
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.37), 0.1)
        assert t.linear.x == 0.0

    def test_normal_mode_pass(self):
        self.g.set_narrow_mode(False)
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.80), 0.1)
        assert t.linear.x > 0.0
