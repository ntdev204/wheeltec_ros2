import math


class ObstacleGuard:
    def __init__(
        self,
        stop_distance: float = 0.30,
        slow_distance: float = 0.60,
        scan_stale_timeout: float = 0.75,
    ):
        self.stop_distance = float(stop_distance)
        self.slow_distance = float(slow_distance)
        self.scan_stale_timeout = float(scan_stale_timeout)

    def guard(self, twist, lidar_sectors, scan_age_s: float):
        """Clamp a Twist-like object using [front, rear, left, right] sector distances."""
        if scan_age_s > self.scan_stale_timeout or lidar_sectors is None:
            self._zero(twist)
            return twist

        front, rear, left, right = self._normalise_sectors(lidar_sectors)

        if twist.linear.x > 0.0:
            twist.linear.x *= self._scale(front)
        elif twist.linear.x < 0.0:
            twist.linear.x *= self._scale(rear)

        if twist.linear.y > 0.0:
            twist.linear.y *= self._scale(left)
        elif twist.linear.y < 0.0:
            twist.linear.y *= self._scale(right)

        if abs(twist.linear.x) < 1e-4:
            twist.linear.x = 0.0
        if abs(twist.linear.y) < 1e-4:
            twist.linear.y = 0.0
        if twist.linear.x == 0.0 and twist.linear.y == 0.0:
            twist.angular.z = 0.0

        return twist

    def _scale(self, distance: float) -> float:
        if distance <= self.stop_distance:
            return 0.0
        if distance >= self.slow_distance:
            return 1.0
        span = max(self.slow_distance - self.stop_distance, 1e-6)
        return max(0.0, min(1.0, (distance - self.stop_distance) / span))

    @staticmethod
    def _normalise_sectors(lidar_sectors) -> tuple[float, float, float, float]:
        values = list(lidar_sectors[:4])
        while len(values) < 4:
            values.append(0.0)
        return tuple(ObstacleGuard._valid_distance(v) for v in values)

    @staticmethod
    def _valid_distance(value) -> float:
        try:
            dist = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(dist) or dist <= 0.0:
            return 0.0
        return dist

    @staticmethod
    def _zero(twist) -> None:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
