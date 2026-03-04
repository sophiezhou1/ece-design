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
    event detection markers for overtemp and add food
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
