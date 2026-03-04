from collections import deque
import matplotlib.pyplot as plt

MAX_POINTS = 500

_t_buf = deque(maxlen=MAX_POINTS)
_raw_buf = deque(maxlen=MAX_POINTS)
_tc_buf = deque(maxlen=MAX_POINTS)

_t0 = None

fig, (_ax_raw, _ax_tc) = plt.subplots(2, 1)

_line_raw, = _ax_raw.plot([], [])
_line_tc, = _ax_tc.plot([], [])

_ax_raw.set_ylabel("Raw ADC")
_ax_tc.set_ylabel("TC Temp (C)")
_ax_tc.set_xlabel("Time (s)")

def update_plot(t_us: int, raw: int, tc: float):
    global _t0

    if _t0 is None:
        _t0 = t_us

    t_s = (t_us - _t0) * 1e-6

    _t_buf.append(t_s)
    _raw_buf.append(raw)
    _tc_buf.append(tc)

    _line_raw.set_data(_t_buf, _raw_buf)
    _line_tc.set_data(_t_buf, _tc_buf)

    # y autoscale only
    _ax_raw.relim()
    _ax_raw.autoscale_view(scalex=False, scaley=True)

    _ax_tc.relim()
    _ax_tc.autoscale_view(scalex=False, scaley=True)

    # scroll x window
    if len(_t_buf) >= 2:
        t_min = _t_buf[0]
        t_max = _t_buf[-1]
        _ax_raw.set_xlim(t_min, t_max)
        _ax_tc.set_xlim(t_min, t_max)

    plt.pause(0.001)