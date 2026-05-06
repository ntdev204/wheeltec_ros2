"""Tests for magnetic repulsion ObstacleGuard."""
from __future__ import annotations
import math, sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from context_aware_bridge.obstacle_guard import ObstacleGuard


def twist(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(linear=SimpleNamespace(x=x, y=y), angular=SimpleNamespace(z=z))

def clear(d=9.9): return [d] * 360

def wall_at(deg, dist, d=9.9):
    s = clear(d); s[deg % 360] = dist; return s


class TestMagneticGuard:
    def setup_method(self):
        self.g = ObstacleGuard(stop_radius=0.50, slow_radius=0.70,
                               influence_radius=1.0, lidar_offset_m=0.10,
                               scan_stale_timeout=0.75)

    # ── Repulsion pushes robot ngược chiều vật cản ────────────────────────────

    def test_wall_ahead_reduces_forward_speed(self):
        """Tường trước 0.8m → lực đẩy ngược → vx_final < vx_commanded."""
        t = self.g.guard(twist(x=0.5), wall_at(0, 0.8), 0.1)
        assert t.linear.x < 0.5, "Repulsion should reduce forward speed"

    def test_wall_right_does_not_create_side_motion_without_cmd(self):
        """Không có lệnh vy thì guard không tự sinh trượt ngang."""
        t = self.g.guard(twist(x=0.3), wall_at(270, 0.8), 0.1)
        assert t.linear.y == 0.0, "No commanded vy -> no lateral motion"

    def test_wall_left_does_not_create_side_motion_without_cmd(self):
        """Không có lệnh vy thì guard không tự sinh trượt ngang."""
        t = self.g.guard(twist(x=0.3), wall_at(90, 0.8), 0.1)
        assert t.linear.y == 0.0, "No commanded vy -> no lateral motion"

    def test_robot_idle_stays_stopped(self):
        """Robot đứng yên thì phải đứng yên, không tự sinh chuyển động tránh."""
        t = self.g.guard(twist(x=0.0), wall_at(0, 0.8), 0.1)
        assert t.linear.x == 0.0, "Stationary robot should remain stopped"

    def test_repulsion_max_speed_capped(self):
        """Tổng lực đẩy không vượt 0.2 m/s."""
        # Tường ở khắp nơi gần → lực lớn nhưng bị cap
        s = [0.4] * 360
        t = self.g.guard(twist(x=0.0), s, 0.1)
        mag = math.sqrt(t.linear.x**2 + t.linear.y**2)
        assert mag <= 0.20 + 0.01, f"Repulsion exceeded max: {mag}"

    def test_symmetric_walls_cancel(self):
        """Tường đều hai bên (trái+phải) → lực đẩy triệt tiêu vy ≈ 0."""
        s = clear()
        s[90] = s[270] = 0.8
        t = self.g.guard(twist(x=0.5), s, 0.1)
        assert abs(t.linear.y) < 0.05, f"Symmetric walls should cancel vy, got {t.linear.y}"

    def test_far_obstacle_no_repulsion(self):
        """Vật cản > influence_radius (1.5m) → không có lực đẩy."""
        t = self.g.guard(twist(x=0.5), wall_at(0, 1.5), 0.1)
        assert abs(t.linear.x - 0.5) < 0.01, "No repulsion beyond influence_radius"

    # ── Hard stop khi vi phạm stop_radius ─────────────────────────────────────

    def test_hard_stop_forward(self):
        """eff = 0.45 - 0.10 = 0.35m < 0.55 → vx = 0."""
        t = self.g.guard(twist(x=0.5), wall_at(0, 0.45), 0.1)
        assert t.linear.x == 0.0

    def test_hard_stop_blocks_repulsion_into_wall(self):
        """Repulsion từ tường sau đẩy robot tiến, nhưng tường trước trong stop zone → tiến bị chặn.
        Repulsion từ tường trước đẩy ngược lại (lùi) → vx < 0 nhưng lùi được vì sau trống.
        Kết quả: vx != dương (không đi vào tường trước).
        """
        s = clear()
        s[180] = 0.8   # tường sau → đẩy tiến (rep_vx > 0)
        s[0]   = 0.40  # tường trước eff=0.30 < 0.55 → tiến bị chặn
        t = self.g.guard(twist(x=0.0), s, 0.1)
        # Robot không được đi tiến vào tường trước
        assert t.linear.x <= 0.0, f"Should not move forward into front wall, got {t.linear.x}"

    # ── Narrow mode ────────────────────────────────────────────────────────────

    def test_narrow_mode_tighter_stop(self):
        """Narrow: stop=0.30m. Lidar 0.37m → eff=0.27m < 0.35m → dừng."""
        self.g.set_narrow_mode(True)
        t = self.g.guard(twist(x=0.5), wall_at(0, 0.37), 0.1)
        assert t.linear.x == 0.0

    # ── Fail-safe ──────────────────────────────────────────────────────────────

    def test_stale_scan(self):
        t = self.g.guard(twist(x=0.5, y=0.3, z=0.8), clear(), 2.0)
        assert t.linear.x == 0.0 and t.linear.y == 0.0 and t.angular.z == 0.0

    def test_none_scan(self):
        assert self.g.guard(twist(x=0.5), None, 0.1).linear.x == 0.0
