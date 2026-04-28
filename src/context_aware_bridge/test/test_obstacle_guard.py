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


class TestObstacleGuard:
    def test_forward_veto(self):
        guard = ObstacleGuard(stop_distance=0.3, slow_distance=0.6)
        twist = guard.guard(make_twist(x=0.5), [0.2, 9.9, 9.9, 9.9], scan_age_s=0.1)
        assert twist.linear.x == 0.0

    def test_reverse_veto(self):
        guard = ObstacleGuard(stop_distance=0.3, slow_distance=0.6)
        twist = guard.guard(make_twist(x=-0.5), [9.9, 0.2, 9.9, 9.9], scan_age_s=0.1)
        assert twist.linear.x == 0.0

    def test_strafe_left_veto(self):
        guard = ObstacleGuard(stop_distance=0.3, slow_distance=0.6)
        twist = guard.guard(make_twist(y=0.5), [9.9, 9.9, 0.2, 9.9], scan_age_s=0.1)
        assert twist.linear.y == 0.0

    def test_slow_down_between_stop_and_slow_distance(self):
        guard = ObstacleGuard(stop_distance=0.3, slow_distance=0.6)
        twist = guard.guard(make_twist(x=0.6), [0.45, 9.9, 9.9, 9.9], scan_age_s=0.1)
        assert 0.0 < twist.linear.x < 0.6

    def test_stale_scan_fail_safe(self):
        guard = ObstacleGuard(stop_distance=0.3, slow_distance=0.6, scan_stale_timeout=0.5)
        twist = guard.guard(make_twist(x=0.5, y=0.2, z=0.3), [9.9, 9.9, 9.9, 9.9], scan_age_s=1.0)
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0
