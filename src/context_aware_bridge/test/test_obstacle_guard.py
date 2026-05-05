"""Tests for stateless radial ObstacleGuard."""
from __future__ import annotations

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
        self.g = ObstacleGuard(stop_radius=0.50, slow_radius=0.70,
                               lidar_offset_m=0.10, scan_stale_timeout=0.75)

    # ── Hard veto ──────────────────────────────────────────────────────────────

    def test_forward_veto(self):
        """Lidar 0.45m phía trước → eff = 0.45 - 0.10 = 0.35m < 0.55 → DỪNG."""
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.45), 0.1)
        assert t.linear.x == 0.0

    def test_forward_pass(self):
        """Lidar 0.90m phía trước → eff = 0.80m > 0.55 → KHÔNG dừng."""
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.90), 0.1)
        assert t.linear.x > 0.0

    def test_rear_veto(self):
        """Lùi, lidar 0.40m phía sau → eff = 0.40 + 0.10 = 0.50 ≤ 0.55 → DỪNG."""
        t = self.g.guard(make_twist(x=-0.5), scan_with(180, 0.40), 0.1)
        assert t.linear.x == 0.0

    def test_strafe_left_veto(self):
        t = self.g.guard(make_twist(y=0.5), scan_with(90, 0.45), 0.1)
        assert t.linear.y == 0.0

    def test_strafe_right_veto(self):
        t = self.g.guard(make_twist(y=-0.5), scan_with(270, 0.45), 0.1)
        assert t.linear.y == 0.0

    def test_obstacle_behind_no_affect_forward(self):
        """Vật cản phía sau không ảnh hưởng khi tiến."""
        t = self.g.guard(make_twist(x=0.5), scan_with(180, 0.10), 0.1)
        assert t.linear.x > 0.0

    def test_obstacle_side_no_affect_forward(self):
        """Vật cản bên hông không ảnh hưởng khi tiến thẳng."""
        t = self.g.guard(make_twist(x=0.5), scan_with(90, 0.10), 0.1)
        assert t.linear.x > 0.0

    def test_obstacle_in_cone_triggers_veto(self):
        """Vật cản ở 20° (trong cone ±30°) → veto tiến."""
        t = self.g.guard(make_twist(x=0.5), scan_with(20, 0.40), 0.1)
        assert t.linear.x == 0.0

    def test_obstacle_outside_cone_no_veto(self):
        """Vật cản ở 35° (ngoài cone ±30°) → không veto."""
        t = self.g.guard(make_twist(x=0.5), scan_with(35, 0.10), 0.1)
        assert t.linear.x >= 0.0

    # ── Soft slow-down ─────────────────────────────────────────────────────────

    def test_soft_slowdown(self):
        """Giữa stop và slow radius → giảm tốc tuyến tính (0 < x < full)."""
        # eff = 0.72 - 0.10 = 0.62m; 0.55 < 0.62 < 0.70 → scale
        t = self.g.guard(make_twist(x=0.6), scan_with(0, 0.72), 0.1)
        assert 0.0 < t.linear.x < 0.6

    # ── Both axes vetoed → angular zero ────────────────────────────────────────

    def test_angular_zeroed_when_both_axes_blocked(self):
        scan = scan_with(0, 0.20)
        scan[270] = 0.20
        t = self.g.guard(make_twist(x=0.5, y=-0.3, z=1.0), scan, 0.1)
        assert t.linear.x == 0.0
        assert t.linear.y == 0.0
        assert t.angular.z == 0.0

    # ── Fail-safe ──────────────────────────────────────────────────────────────

    def test_stale_scan(self):
        t = self.g.guard(make_twist(x=0.5, y=0.3, z=0.8), scan_clear(), 2.0)
        assert t.linear.x == 0.0 and t.linear.y == 0.0 and t.angular.z == 0.0

    def test_null_scan(self):
        t = self.g.guard(make_twist(x=0.5), None, 0.1)
        assert t.linear.x == 0.0

    # ── Narrow mode ────────────────────────────────────────────────────────────

    def test_narrow_mode_tighter(self):
        """Narrow: stop=0.30m. Lidar 0.37m → eff = 0.27m < 0.35 → DỪNG."""
        self.g.set_narrow_mode(True)
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.37), 0.1)
        assert t.linear.x == 0.0

    def test_normal_mode_restored(self):
        """Normal: stop=0.50m. Lidar 0.80m → eff = 0.70m > 0.55 → pass."""
        self.g.set_narrow_mode(False)
        t = self.g.guard(make_twist(x=0.5), scan_with(0, 0.80), 0.1)
        assert t.linear.x > 0.0
