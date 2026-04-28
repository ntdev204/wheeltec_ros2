
import unittest
from cmath import sqrt
from nav2_simple_commander.line_iterator import LineIterator


class TestLineIterator(unittest.TestCase):

    def test_type_error(self):
        self.assertRaises(TypeError, LineIterator, 0, 0, '10', 10, '1')

    def test_value_error(self):
        self.assertRaises(ValueError, LineIterator, 0, 0, 10, 10, -2)
        self.assertRaises(ValueError, LineIterator, 2, 2, 2, 2, 1)

    def test_get_xy(self):
        lt = LineIterator(0, 0, 5, 5, 1)
        self.assertEqual(lt.getX0(), 0)
        self.assertEqual(lt.getY0(), 0)
        self.assertEqual(lt.getX1(), 5)
        self.assertEqual(lt.getY1(), 5)

    def test_line_length(self):
        lt = LineIterator(0, 0, 5, 5, 1)
        self.assertEqual(lt.get_line_length(), sqrt(pow(5, 2) + pow(5, 2)))

    def test_straight_line(self):
        lt = LineIterator(0, 0, 5, 5, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1

        lt = LineIterator(0, 0, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + (i*2))
            lt.advance()
            i += 1

        lt = LineIterator(0, 0, 5, -10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + (-i*2))
            lt.advance()
            i += 1

    def test_hor_line(self):
        lt = LineIterator(0, 10, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0())
            lt.advance()
            i += 1

    def test_ver_line(self):
        lt = LineIterator(5, 0, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0())
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1

    def test_clamp(self):
        lt = LineIterator(0, 0, 5, 5, 10)
        self.assertEqual(lt.getX(), 0)
        self.assertEqual(lt.getY(), 0)
        lt.advance()
        while lt.isValid():
            self.assertEqual(lt.getX(), 5)
            self.assertEqual(lt.getY(), 5)
            lt.advance()


if __name__ == '__main__':
    unittest.main()
