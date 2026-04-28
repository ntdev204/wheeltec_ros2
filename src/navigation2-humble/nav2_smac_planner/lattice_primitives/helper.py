
import numpy as np


def normalize_angle(angle):
    while angle >= 2*np.pi:
        angle -= 2*np.pi

    while angle < 0:
        angle += 2*np.pi

    return angle


def angle_difference(angle_1, angle_2, left_turn=None):
    if left_turn is None:
        dif = abs(angle_1 - angle_2)

        return dif if dif <= np.pi else 2 * np.pi - dif

    elif left_turn:

        if angle_2 >= angle_1:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)

    else:
        if angle_1 >= angle_2:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)


def interpolate_yaws(start_angle, end_angle, left_turn, steps):
    if left_turn:
        if start_angle > end_angle:
            end_angle += 2 * np.pi
    else:
        if end_angle > start_angle:
            end_angle -= 2 * np.pi

    yaws = np.linspace(start_angle, end_angle, steps)
    yaws = np.vectorize(normalize_angle)(yaws)

    return yaws


def get_rotation_matrix(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])
