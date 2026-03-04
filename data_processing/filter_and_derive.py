"""
filter data for smoothing, derive slope and variance
    variance for fsm.py

smoothing:
    moving average filter
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
import math

_prev_adc_temp = None
def thermistor_conv(raw: int):
    global _prev_adc_temp

    ADC_MAX = 4095
    rfixed = 100000
    vcc = 3.295

    try:
        raw = max(1, min(ADC_MAX - 1, int(raw)))  # avoid endpoints

        vout = raw * vcc / ADC_MAX
        rth = vout * rfixed / (vcc - vout)

        A = -0.1943e-3
        B = 3.4023e-4
        C = -2.3843e-7

        ln = math.log(rth)
        Tinv = A + B * ln + C * (ln ** 3)
        T = (1.0 / Tinv) - 273.15

        _prev_adc_temp = T
        return T
    except Exception:
        return _prev_adc_temp

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