import math
from typing import List, Dict, Tuple, Optional

import numpy as np
from nav_msgs.msg import OccupancyGrid


class CoveragePlanner:

    def __init__(self, robot_width: float = 0.5, overlap: float = 0.1):
        self.robot_width = robot_width
        self.overlap = overlap
        self.min_clearance = 0.3

    def generate_coverage_waypoints(
        self,
        map_msg: OccupancyGrid,
        pattern: str = "boustrophedon"
    ) -> List[Dict[str, float]]:
        if pattern == "boustrophedon":
            return self._generate_boustrophedon(map_msg)
        elif pattern == "spiral":
            return self._generate_spiral(map_msg)
        else:
            raise ValueError(f"Unknown pattern type: {pattern}")

    def _generate_boustrophedon(self, map_msg: OccupancyGrid) -> List[Dict[str, float]]:
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        grid = np.array(map_msg.data).reshape((height, width))

        stripe_width_m = self.robot_width * (1.0 - self.overlap)
        stripe_width_cells = max(1, int(stripe_width_m / resolution))

        inflation_cells = int((self.robot_width / 2.0 + self.min_clearance) / resolution)
        inflated_grid = self._inflate_obstacles(grid, inflation_cells)

        waypoints = []
        y_cell = 0
        direction = 1

        while y_cell < height:
            row = inflated_grid[y_cell, :]
            free_segments = self._find_free_segments(row)

            if free_segments:
                y_world = origin_y + (y_cell + 0.5) * resolution

                for x_start, x_end in free_segments:
                    if direction == 1:
                        x_world_start = origin_x + (x_start + 0.5) * resolution
                        x_world_end = origin_x + (x_end - 0.5) * resolution
                        yaw = 0.0
                    else:
                        x_world_start = origin_x + (x_end - 0.5) * resolution
                        x_world_end = origin_x + (x_start + 0.5) * resolution
                        yaw = math.pi

                    waypoints.append({
                        "x": float(x_world_start),
                        "y": float(y_world),
                        "yaw": float(yaw)
                    })

                    waypoints.append({
                        "x": float(x_world_end),
                        "y": float(y_world),
                        "yaw": float(yaw)
                    })

                direction *= -1

            y_cell += stripe_width_cells

        optimized = self._optimize_waypoints(waypoints)

        return optimized

    def _generate_spiral(self, map_msg: OccupancyGrid) -> List[Dict[str, float]]:
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        center_x = width // 2
        center_y = height // 2

        waypoints = []

        layer = 1
        max_layer = max(width, height) // 2

        while layer <= max_layer:
            for x in range(center_x - layer, center_x + layer + 1):
                if 0 <= x < width and 0 <= center_y - layer < height:
                    x_world = origin_x + (x + 0.5) * resolution
                    y_world = origin_y + (center_y - layer + 0.5) * resolution
                    waypoints.append({"x": float(x_world), "y": float(y_world), "yaw": 0.0})

            for y in range(center_y - layer, center_y + layer + 1):
                if 0 <= center_x + layer < width and 0 <= y < height:
                    x_world = origin_x + (center_x + layer + 0.5) * resolution
                    y_world = origin_y + (y + 0.5) * resolution
                    waypoints.append({"x": float(x_world), "y": float(y_world), "yaw": math.pi / 2})

            for x in range(center_x + layer, center_x - layer - 1, -1):
                if 0 <= x < width and 0 <= center_y + layer < height:
                    x_world = origin_x + (x + 0.5) * resolution
                    y_world = origin_y + (center_y + layer + 0.5) * resolution
                    waypoints.append({"x": float(x_world), "y": float(y_world), "yaw": math.pi})

            for y in range(center_y + layer, center_y - layer - 1, -1):
                if 0 <= center_x - layer < width and 0 <= y < height:
                    x_world = origin_x + (center_x - layer + 0.5) * resolution
                    y_world = origin_y + (y + 0.5) * resolution
                    waypoints.append({"x": float(x_world), "y": float(y_world), "yaw": -math.pi / 2})

            layer += 1

        return waypoints

    def _inflate_obstacles(self, grid: np.ndarray, inflation_cells: int) -> np.ndarray:
        from scipy.ndimage import binary_dilation

        obstacles = (grid == -1) | (grid > 50)

        if inflation_cells > 0:
            structure = np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1))
            inflated = binary_dilation(obstacles, structure=structure)
        else:
            inflated = obstacles

        return inflated.astype(np.uint8) * 100

    def _find_free_segments(self, row: np.ndarray, min_length: int = 3) -> List[Tuple[int, int]]:
        segments = []
        in_segment = False
        start = 0

        for i, cell in enumerate(row):
            if cell == 0:
                if not in_segment:
                    start = i
                    in_segment = True
            else:
                if in_segment:
                    if i - start >= min_length:
                        segments.append((start, i))
                    in_segment = False

        if in_segment and len(row) - start >= min_length:
            segments.append((start, len(row)))

        return segments

    def _optimize_waypoints(self, waypoints: List[Dict[str, float]]) -> List[Dict[str, float]]:
        if len(waypoints) < 3:
            return waypoints

        optimized = [waypoints[0]]

        for i in range(1, len(waypoints) - 1):
            prev = waypoints[i - 1]
            curr = waypoints[i]
            next_wp = waypoints[i + 1]

            dx1 = curr["x"] - prev["x"]
            dy1 = curr["y"] - prev["y"]
            dx2 = next_wp["x"] - curr["x"]
            dy2 = next_wp["y"] - curr["y"]

            cross = dx1 * dy2 - dy1 * dx2

            if abs(cross) > 0.01:
                optimized.append(curr)

        optimized.append(waypoints[-1])

        return optimized

    def validate_waypoints(
        self,
        waypoints: List[Dict[str, float]],
        map_msg: OccupancyGrid
    ) -> Tuple[bool, Optional[str]]:
        if len(waypoints) < 2:
            return False, "Coverage route must have at least 2 waypoints"

        if len(waypoints) > 1000:
            return False, f"Coverage route has too many waypoints ({len(waypoints)}), max 1000"

        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        for i, wp in enumerate(waypoints):
            x_cell = int((wp["x"] - origin_x) / resolution)
            y_cell = int((wp["y"] - origin_y) / resolution)

            if not (0 <= x_cell < width and 0 <= y_cell < height):
                return False, f"Waypoint {i} is outside map bounds"

            grid_index = y_cell * width + x_cell
            if grid_index < len(map_msg.data):
                cell_value = map_msg.data[grid_index]
                if cell_value > 50 or cell_value == -1:
                    return False, f"Waypoint {i} is in occupied or unknown space"

        return True, None
