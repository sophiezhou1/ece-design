"""
filter data for smoothing, derive slope and variance
    variance for fsm.py

filters:
    moving average filter
    median filtering for spikes

features:
    indicate temp increase rate
    variance
    time to target temp prediction
"""


from collections import deque
import statistics
    
# filters
class MedianFilter:
    def __init__(self, N):
        self.buf = deque(maxlen=N)

    def update(self, x):
        self.buf.append(x)
        return statistics.median(self.buf)

class MovingAverage:
    def __init__(self, N: int):
        self.N = N
        self.buf = deque(maxlen=N)
        self.s = 0.0

    # append to buffer (buffer.update), return current average
    def update(self, x: float):
        if x is None:
            return self.s / len(self.buf) if self.buf else None

        if len(self.buf) == self.N:
            self.s -= self.buf[0]
        self.buf.append(x)
        self.s += x
        return self.s / len(self.buf)
    

# features
class MovingLinearRegression:
    def __init__(self, window_size, min_points = 2, epsilon = 1e-12):
        self.Nmax = window_size
        self.min_points = min_points
        self.epsilon = epsilon

        self.buf = deque(maxlen=window_size)

        self.n = 0
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_x2 = 0.0
        self.sum_xy = 0.0

    def reset(self) -> None:
        self.buf.clear()
        self.n = 0
        self.sum_x = self.sum_y = self.sum_x2 = self.sum_xy = 0.0

    def update(self, x, y):
        if self.n == self.Nmax:
            x0, y0 = self.buf[0]
            self.sum_x -= x0
            self.sum_y  -= y0
            self.sum_x2 -= x0 * x0
            self.sum_xy -= x0 * y0
        else:
            self.n += 1

        self.buf.append((x, y))
        self.sum_x  += x
        self.sum_y  += y
        self.sum_x2 += x * x
        self.sum_xy += x * y

        if self.n < self.min_points:
            return None

        denom = self.n * self.sum_x2 - (self.sum_x * self.sum_x)
        if abs(denom) < self.epsilon:
            return None

        slope = (self.n * self.sum_xy - self.sum_x * self.sum_y) / denom
        return slope