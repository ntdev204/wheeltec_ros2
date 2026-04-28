
import logging
from typing import Tuple, Union

import numpy as np

from trajectory import Path, Trajectory, TrajectoryParameters

logger = logging.getLogger(__name__)


class TrajectoryGenerator:

    def __init__(self, config: dict):
        self.turning_radius = config['turning_radius']

    def _get_arc_point(
        self, trajectory_params: TrajectoryParameters, t: float
    ) -> Tuple[float, float, float]:
        start_angle = trajectory_params.start_angle

        arc_dist = t * trajectory_params.arc_length
        angle_step = arc_dist / trajectory_params.turning_radius

        if trajectory_params.left_turn:
            t = start_angle + angle_step
            x = (
                trajectory_params.turning_radius * np.cos(t - np.pi / 2)
                + trajectory_params.x_offset
            )
            y = (
                trajectory_params.turning_radius * np.sin(t - np.pi / 2)
                + trajectory_params.y_offset
            )

            yaw = t

        else:
            start_angle = -start_angle

            t = start_angle + angle_step
            x = (
                trajectory_params.turning_radius * -np.cos(t + np.pi / 2)
                + trajectory_params.x_offset
            )
            y = (
                trajectory_params.turning_radius * np.sin(t + np.pi / 2)
                + trajectory_params.y_offset
            )

            yaw = -t

        return x, y, yaw

    def _get_line_point(
        self, start_point: np.array, end_point: np.array, t: float
    ) -> Tuple[float, float]:
        return start_point + t * (end_point - start_point)

    def _create_path(
        self, trajectory_params: TrajectoryParameters, primitive_resolution: float
    ) -> Path:
        number_of_steps = np.round(
            trajectory_params.total_length / primitive_resolution
        ).astype(int)
        t_step = 1 / number_of_steps

        start_to_arc_dist = np.linalg.norm(trajectory_params.arc_start_point)

        transition_points = [
            start_to_arc_dist / trajectory_params.total_length,
            (start_to_arc_dist + trajectory_params.arc_length)
            / trajectory_params.total_length,
        ]

        cur_t = t_step

        xs = []
        ys = []
        yaws = []

        for i in range(1, number_of_steps + 1):

            cur_t = min(cur_t, 1)

            if cur_t <= transition_points[0]:
                line_t = cur_t / transition_points[0]
                x, y = self._get_line_point(
                    np.array([0, 0]), trajectory_params.arc_start_point, line_t
                )
                yaw = trajectory_params.start_angle

            elif cur_t <= transition_points[1]:
                arc_t = (cur_t - transition_points[0]) / (
                    transition_points[1] - transition_points[0]
                )
                x, y, yaw = self._get_arc_point(trajectory_params, arc_t)

            else:
                line_t = (cur_t - transition_points[1]) / (1 - transition_points[1])
                x, y = self._get_line_point(
                    trajectory_params.arc_end_point, trajectory_params.end_point, line_t
                )
                yaw = trajectory_params.end_angle

            xs.append(x)
            ys.append(y)
            yaws.append(yaw)

            cur_t += t_step

        xs = np.array(xs)
        ys = np.array(ys)
        yaws = np.array(yaws)

        xs[-1], ys[-1] = trajectory_params.end_point
        yaws[-1] = trajectory_params.end_angle

        return Path(xs, ys, yaws)

    def _get_intersection_point(
        self, m1: float, c1: float, m2: float, c2: float
    ) -> np.array:
        def line1(x):
            return m1 * x + c1

        x_point = (c2 - c1) / (m1 - m2)

        return np.array([x_point, line1(x_point)])

    def _is_left_turn(self, intersection_point: np.array, end_point: np.array) -> bool:
        matrix = np.vstack([intersection_point, end_point])
        det = np.linalg.det(matrix)

        return det >= 0

    def _is_dir_vec_correct(
        self, point1: np.array, point2: np.array, line_angle: float
    ) -> bool:
        m = abs(np.tan(line_angle).round(5))

        if line_angle < 0:
            m *= -1

        direction_vec_from_points = point2 - point1

        direction_vec_from_gradient = np.array([1, m])

        if abs(line_angle) > np.pi / 2:
            direction_vec_from_gradient = np.array([-1, m])
        elif abs(line_angle) == np.pi / 2:
            direction_vec_from_gradient = np.array([0, m])

        direction_vec_from_gradient = direction_vec_from_gradient.round(5)
        direction_vec_from_points = direction_vec_from_points.round(5)

        if np.all(
            np.sign(direction_vec_from_points) == np.sign(direction_vec_from_gradient)
        ):
            return True
        else:
            return False

    def _calculate_trajectory_params(
        self, end_point: np.array, start_angle: float, end_angle: float
    ) -> Union[TrajectoryParameters, None]:
        x2, y2 = end_point
        arc_start_point = np.array([0, 0])
        arc_end_point = end_point

        m1 = np.tan(start_angle).round(5)

        m2 = np.tan(end_angle).round(5)

        if m1 == m2:
            if round(-m2 * x2 + y2, 5) == 0:
                return TrajectoryParameters.no_arc(
                    end_point=end_point, start_angle=start_angle, end_angle=end_angle
                )

            elif (
                abs(start_angle) == np.pi / 2 and arc_end_point[0] == arc_start_point[0]
            ):
                return TrajectoryParameters.no_arc(
                    end_point=end_point,
                    start_angle=start_angle,
                    end_angle=end_angle,
                )

            else:
                logger.debug(
                    'No trajectory possible for equivalent start and '
                    + f'end angles that also passes through p = {x2, y2}'
                )
                return None

        intersection_point = self._get_intersection_point(m1, 0, m2, -m2 * x2 + y2)

        if not self._is_dir_vec_correct(
            arc_start_point, intersection_point, start_angle
        ):
            logger.debug(
                'No trajectory possible since intersection point occurs '
                + 'before start point on line 1'
            )
            return None

        if not self._is_dir_vec_correct(intersection_point, arc_end_point, end_angle):
            logger.debug(
                'No trajectory possible since intersection point occurs '
                + 'after end point on line 2'
            )
            return None

        dist_a = round(np.linalg.norm(arc_start_point - intersection_point), 5)

        dist_b = round(np.linalg.norm(arc_end_point - intersection_point), 5)

        angle_between_lines = np.pi - abs(end_angle - start_angle)

        min_valid_distance = round(
            self.turning_radius / np.tan(angle_between_lines / 2), 5
        )

        if dist_a < min_valid_distance or dist_b < min_valid_distance:
            logger.debug(
                'No trajectory possible where radius is larger than '
                + 'minimum turning radius'
            )
            return None

        if dist_a < dist_b:
            vec_line2 = arc_end_point - intersection_point
            vec_line2 /= np.linalg.norm(vec_line2)
            arc_end_point = intersection_point + dist_a * vec_line2

        elif dist_a > dist_b:
            vec_line1 = arc_start_point - intersection_point
            vec_line1 /= np.linalg.norm(vec_line1)

            arc_start_point = intersection_point + dist_b * vec_line1

        x1, y1 = arc_start_point
        x2, y2 = arc_end_point

        if m1 == 0:

            def perp_line2(x):
                return -1 / m2 * (x - x2) + y2

            circle_center = np.array([x1, perp_line2(x1)])
        elif m2 == 0:

            def perp_line1(x):
                return -1 / m1 * (x - x1) + y1

            circle_center = np.array([x2, perp_line1(x2)])
        else:
            perp_m1 = -1 / m1 if m1 != 0 else 0
            perp_m2 = -1 / m2 if m2 != 0 else 0

            circle_center = self._get_intersection_point(
                perp_m1, -perp_m1 * x1 + y1, perp_m2, -perp_m2 * x2 + y2
            )

        radius = np.linalg.norm(circle_center - arc_end_point).round(5)
        x_offset = circle_center[0].round(5)
        y_offset = circle_center[1].round(5)

        if radius < self.turning_radius:
            logger.debug(
                'Calculated circle radius is smaller than allowed turning '
                + f'radius: r = {radius}, min_radius = {self.turning_radius}'
            )
            return None

        left_turn = self._is_left_turn(intersection_point, end_point)

        return TrajectoryParameters(
            radius,
            x_offset,
            y_offset,
            end_point,
            start_angle,
            end_angle,
            left_turn,
            arc_start_point,
            arc_end_point,
        )

    def generate_trajectory(
        self,
        end_point: np.array,
        start_angle: float,
        end_angle: float,
        primitive_resolution: float,
    ) -> Union[Trajectory, None]:
        trajectory_params = self._calculate_trajectory_params(
            end_point, start_angle, end_angle
        )

        if trajectory_params is None:
            return None

        logger.debug('Trajectory found')

        trajectory_path = self._create_path(trajectory_params, primitive_resolution)

        return Trajectory(trajectory_path, trajectory_params)
