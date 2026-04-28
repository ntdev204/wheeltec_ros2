

from cmath import sqrt


class LineIterator():

    def __init__(self, x0, y0, x1, y1, step_size=1.0):
        if type(x0) not in [int, float]:
            raise TypeError("x0 must be a number (int or float)")

        if type(y0) not in [int, float]:
            raise TypeError("y0 must be a number (int or float)")

        if type(x1) not in [int, float]:
            raise TypeError("x1 must be a number (int or float)")

        if type(y1) not in [int, float]:
            raise TypeError("y1 must be a number (int or float)")

        if type(step_size) not in [int, float]:
            raise TypeError("step_size must be a number (int or float)")

        if step_size <= 0:
            raise ValueError("step_size must be a positive number")

        self.x0_ = x0
        self.y0_ = y0
        self.x1_ = x1
        self.y1_ = y1
        self.x_ = x0
        self.y_ = y0
        self.step_size_ = step_size

        if x1 != x0 and y1 != y0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        elif x1 == x0 and y1 != y0:
            self.valid_ = True
        elif y1 == y1 and x1 != x0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        else:
            self.valid_ = False
            raise ValueError(
                "Line has zero length (All 4 points have same coordinates)")

    def isValid(self):
        return self.valid_

    def advance(self):
        if self.x1_ > self.x0_:
            if self.x_ < self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ + self.step_size_, self.x0_, self.x1_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        elif self.x1_ < self.x0_:
            if self.x_ > self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ - self.step_size_, self.x1_, self.x0_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        else:
            if self.y1_ > self.y0_:
                if self.y_ < self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ + self.step_size_, self.y0_, self.y1_), 5)
                else:
                    self.valid_ = False
            elif self.y1_ < self.y0_:
                if self.y_ > self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ - self.step_size_, self.y1_, self.y0_), 5)
                else:
                    self.valid_ = False
            else:
                self.valid_ = False

    def getX(self):
        return self.x_

    def getY(self):
        return self.y_

    def getX0(self):
        return self.x0_

    def getY0(self):
        return self.y0_

    def getX1(self):
        return self.x1_

    def getY1(self):
        return self.y1_

    def get_line_length(self):
        return sqrt(pow(self.x1_ - self.x0_, 2) + pow(self.y1_ - self.y0_, 2))

    def clamp(self, n, min_n, max_n):
        if n < min_n:
            return min_n
        elif n > max_n:
            return max_n
        else:
            return n
