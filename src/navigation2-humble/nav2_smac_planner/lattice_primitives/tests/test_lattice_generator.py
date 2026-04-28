
import unittest

from lattice_generator import LatticeGenerator
import numpy as np

MOTION_MODEL = 'ackermann'
TURNING_RADIUS = 0.5
GRID_RESOLUTION = 0.05
STOPPING_THRESHOLD = 5
NUM_OF_HEADINGS = 16


class TestLatticeGenerator(unittest.TestCase):

    def setUp(self) -> None:
        config = {'motion_model': MOTION_MODEL,
                  'turning_radius': TURNING_RADIUS,
                  'grid_resolution': GRID_RESOLUTION,
                  'stopping_threshold': STOPPING_THRESHOLD,
                  'num_of_headings': NUM_OF_HEADINGS}

        lattice_gen = LatticeGenerator(config)

        self.minimal_set = lattice_gen.run()

    def test_minimal_set_lengths_are_positive(self):

        for start_angle in self.minimal_set.keys():
            for trajectory in self.minimal_set[start_angle]:

                self.assertGreaterEqual(trajectory.parameters.arc_length, 0)
                self.assertGreaterEqual(trajectory.parameters.start_straight_length, 0)
                self.assertGreaterEqual(trajectory.parameters.end_straight_length, 0)
                self.assertGreaterEqual(trajectory.parameters.total_length, 0)

    def test_minimal_set_end_points_lie_on_grid(self):

        for start_angle in self.minimal_set.keys():
            for trajectory in self.minimal_set[start_angle]:

                end_point_x = trajectory.path.xs[-1]
                end_point_y = trajectory.path.ys[-1]

                div_x = end_point_x / GRID_RESOLUTION
                div_y = end_point_y / GRID_RESOLUTION

                self.assertAlmostEqual(div_x, np.round(div_x), delta=0.00001)
                self.assertAlmostEqual(div_y, np.round(div_y), delta=0.00001)

    def test_minimal_set_end_angle_is_correct(self):

        for start_angle in self.minimal_set.keys():
            for trajectory in self.minimal_set[start_angle]:

                end_point_angle = trajectory.path.yaws[-1]

                self.assertEqual(end_point_angle, trajectory.parameters.end_angle)

    def test_output_angles_in_correct_range(self):

        for start_angle in self.minimal_set.keys():
            for trajectory in self.minimal_set[start_angle]:
                output = trajectory.path.to_output_format()

                for x, y, angle in output:
                    self.assertGreaterEqual(angle, 0)
                    self.assertLessEqual(angle, 2*np.pi)


if __name__ == '__main__':
    unittest.main()
