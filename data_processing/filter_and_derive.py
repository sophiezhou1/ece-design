"""
filter data for smoothing, derive slope and variance
    variance for fsm.py

smoothing (moving average filter)
    median filtering for spikes
range checks (for sensor dropout)
    probe disconnect reading invalid if dropout?
    thermocouple temp low/unchanging if heat on
    interpolate for small gaps
indicate temp increase rate
    time to target temp prediction
    event detection markers for overtemp and add food
"""

"""
class sensor_data:
    t_us: int
    adc: int
    tc: float
"""

from collections import deque

class MovingAverage:
    def __init__(self, N: int):
        self.N = N
        self.buf = deque(maxlen=N)
        self.s = 0.0

    def update(self, x: float):
        if x is None:
            return self.s / len(self.buf) if self.buf else None

        if len(self.buf) == self.N:
            self.s -= self.buf[0]
        self.buf.append(x)
        self.s += x
        return self.s / len(self.buf)

HISTORY_MAX = 5000
history = deque(maxlen=HISTORY_MAX)

MA_N = 25
ma_adc = MovingAverage(MA_N)
ma_tc  = MovingAverage(MA_N)