# NOTE: UPDATING THE GRAPH INCREASES DELAY SIGNIFICANTLY
import sys

import time
import serial
from serial.tools import list_ports
import math
import re
import numpy as np
from dataclasses import dataclass
import time

import matplotlib.pyplot as plt
from collections import deque

# external py files
import plot
import filter_and_derive
import fault_detection
import fsm





def _fmt_mmss(sec: float) -> str:
    sec_i = max(0, int(sec + 0.5))
    m = sec_i // 60
    s = sec_i % 60
    return f"{m:02d}:{s:02d}"

def _countdown_line(label: str, remaining_s: float) -> None:
    sys.stdout.write("\r" + f"[TIMER_MODE] {label} remaining {_fmt_mmss(remaining_s)}" + " " * 10)
    sys.stdout.flush()

def _status_line(pan_c, internal_c):
    msg = (f"pan={pan_c:5.2f}C  internal={internal_c:5.2f}C")

    sys.stdout.write("\r" + msg + " " * 10)
    sys.stdout.flush()



# buffer
HISTORY_MAX = 5000
history = deque(maxlen=HISTORY_MAX)

# update to reduce lag, lower N = less lag
# N is the number of samples to gather and apply one filter to
# alt: increase sample rate or decrease decimation in mcu/main.c
MA_N = 10
MED_N = 3
med_adc = filter_and_derive.MedianFilter(MED_N)
ma_adc = filter_and_derive.MovingAverage(MA_N)
med_tc = filter_and_derive.MedianFilter(MED_N)
ma_tc  = filter_and_derive.MovingAverage(MA_N)

# linear regression
linreg = filter_and_derive.MovingLinearRegression(50, 50)
var = filter_and_derive.MovingVariance(50, 50)

# structs
@dataclass
class SensorData:
    t_us: int
    adc: int
    tc: float

@dataclass
class Properties:
    # for persistent, not single sample
    fault: int = -1
    fault_oc: int = -1
    fault_scg: int = -1
    fault_scv: int = -1

    fault_drop: int = -1
    fault_probe_dc: int = -1
    fault_tc_dc: int = -1

    warn_overtemp: int = -1
    timer_mode: bool = False

    slope: float = math.nan
    var: float = math.nan
    curr_state: str | None = None

properties = Properties()

# serial params
BAUD = 115200
TIMEOUT_S = 1.0

PORT_HINTS = [
    "/dev/cu.usbserial",
    "/dev/cu.SLAB_USBtoUART",
    "/dev/cu.wchusbserial",
    "/dev/cu.usbmodem",
]



def find_port() -> str | None:
    ports = list(list_ports.comports())
    for p in ports:
        dev = p.device
        if any(hint in dev for hint in PORT_HINTS):
            return dev
    return None

def open_serial() -> serial.Serial:
    while True:
        port = find_port()
        if not port:
            print("No serial device found. Plug in the board and try again...")
            time.sleep(1.0)
            continue

        try:
            s = serial.Serial(port, BAUD, timeout=TIMEOUT_S)
            try:
                s.reset_input_buffer()
            except Exception:
                pass
            print(f"Connected: {port} @ {BAUD}")
            return s
        except serial.SerialException as e:
            print(f"Could not open {port}: {e}")
            time.sleep(1.0)

def parse_line(line: str):
    try:
        # split timestamp + rest of the text
        t_us_str, rest = line.split(",", 1)
        t_us = int(t_us_str.strip())

        # rest
        adc_match = re.search(r"adc(\d+)", line)
        raw_match = re.search(r"raw=0x([0-9A-Fa-f]+)", rest)
        tc_match = re.search(r"tc=([-+]?\d*\.?\d+)", rest)
        intern_match = re.search(r"intern=([-+]?\d*\.?\d+)", rest)
        fault_match = re.search(r"fault=(\d+)", rest)
        oc_match = re.search(r"OC=(\d+)", rest)
        scg_match = re.search(r"SCG=(\d+)", rest)
        scv_match = re.search(r"SCV=(\d+)", rest)

        if not all([adc_match, raw_match, tc_match, fault_match, oc_match, scg_match, scv_match]):
            # print(raw_match, tc_match, fault_match, oc_match, scg_match, scv_match)
            # print("bc of match")
            # print(line)
            return None

        adc = int(adc_match.group(1))
        raw = int(raw_match.group(1), 16)
        tc = float(tc_match.group(1))
        intern = float(intern_match.group(1))
        fault = int(fault_match.group(1))
        fault_oc = int(oc_match.group(1))
        fault_scg = int(scg_match.group(1))
        fault_scv = int(scv_match.group(1))

        return (t_us,
                adc,
                raw,
                tc,
                intern,
                fault,
                fault_oc,
                fault_scg,
                fault_scv)

    except Exception as e:
        # print('bc of exception')
        # print(line)
        # print(e)
        return None
    


def c_to_f(c):
    return c * 9.0 / 5.0 + 32

def thermistor_conv(raw):
    ADC_MAX = 4095
    rfixed = 100000
    vcc = 3.295

    # error case for digital output < 1 or equal to 4095 to avoid div by 0
    raw = max(1, min(ADC_MAX - 1, int(raw)))
    vout = raw * vcc / ADC_MAX

    A = -0.1943 * 10 ** -3
    B = 3.4023 * 10 ** -4
    C = -2.3843 * 10 ** -7
    
    rth = vout * rfixed / (vcc - vout)
    Tinv = A + B * math.log(rth) + C * (math.log(rth)) ** 3
    T = Tinv ** -1
    T -= 273.15
    return T #ºC

    # return c_to_f(T)
    # return vout



def main():
    global med_adc, med_tc, ma_adc, ma_tc, properties, linreg
    ser_conn = open_serial()
    cfg = fault_detection.ThresholdConfig(overtemp_set=40, overtemp_clr=38, overtemp_trip_time=0.5)
    monitor = fault_detection.FaultMonitor(cfg)


    started = False
    flip_sent = False
    prev_overtemp = 0

    timer_mode = False
    deadline_s = None
    deadline_label = ""

    PREHEAT_TARGET_C = 30.0
    FLIP_PAN_C       = 40.0
    INTERNAL_TARGET_C = 50.0

    # time-based durations
    SEAR_SIDE_S = 10.0   # 3 min
    REST_S      = 10.0   # 5 min

    fsm_sys = fsm.FSM(target_pan_temp_c=PREHEAT_TARGET_C)


    while True:
        try:
            line = ser_conn.readline()
            if line == b"":
                continue

            text = line.decode(errors="ignore").strip()
            if not text:
                continue

            try:
                # fault flags used later in fault_detection.py for persistent fault detection
                (t_us, adc, _, tc, intern, fault, fault_oc, fault_scg, fault_scv) = parse_line(text)
            except:
                continue
            temp = thermistor_conv(adc)

            # [1] non-filtered outputs
            # print(f"{temp:.2f} {tc:.2f}")
            # plot.update_plot(t_us, temp, tc)

            # [2] with moving avg filtering
            sample = SensorData(t_us, temp, tc) # adc is preprocessed temp
            history.append(sample)
            temp_med = med_adc.update(temp)
            temp_avg = ma_adc.update(temp_med)
            tc_med = med_tc.update(tc)
            tc_avg = ma_tc.update(tc_med)
            # print(f"{temp_avg:.2f} {tc_avg:.2f}")
            # plot.update_plot(t_us, temp_avg, tc_avg)

            # [3] temp slope
            slope = linreg.update(t_us * 1e-6, temp_avg)
            if slope is None:
                continue
            # ºC/min
            properties.slope = slope * 60 # caluculates slope every sample, print out to display at lower rate
            # print(properties.slope)

            # [4] temp var
            variance = var.update(temp_avg)
            if variance is None:
                continue
            properties.var = variance
            # print(properties.var)

            # [5] fault detection
            # cfg and monitor defined at top of main
            # print(adc, tc, intern)
            flags = monitor.update(t_us, adc, tc, intern, fault, fault_oc, fault_scg, fault_scv)
            # print(flags)
            fault_strings = []

            # if flags.fault_probe_dc == 1:
            #     fault_strings.append("PROBE_DISCONNECT")
            # if flags.fault_tc_dc == 1:
            #     fault_strings.append("THERMOCOUPLE_DISCONNECT")
            # if flags.fault_oc == 1:
            #     fault_strings.append("TC_OPEN")
            # if flags.fault_scg == 1:
            #     fault_strings.append("TC_SHORT_GND")
            # if flags.fault_scv == 1:
            #     fault_strings.append("TC_SHORT_VCC")
            # if flags.warn_overtemp == 1:
            #     fault_strings.append("OVERTEMP")
            # if flags.timer_mode:
            #     fault_strings.append("TIMER_MODE")

            # print(f"{temp:.2f} {tc:.2f}")
            fault_detection.apply_fault_flags(properties, flags)
            if properties.fault_probe_dc == 1:
                fault_strings.append("PROBE_DISCONNECT")
            if properties.fault_tc_dc == 1:
                fault_strings.append("THERMOCOUPLE_DISCONNECT")
            # if properties.fault_probe_dc == 1 or properties.fault_tc_dc == 1:
                # print("switching to time-based instructions")
            if properties.warn_overtemp == 1:
                fault_strings.append("OVERTEMP")
                # print("decreasing knob rotation")

            # print(",".join(fault_strings))

            # [6] sample fsm
            # start in IDLE

            # if fsm_sys.state == fsm.State.IDLE:
            #     fsm_sys.dispatch(fsm.EventMsg(fsm.Event.START))

            # fsm_sys.update_pan_temp(tc)

            # if fsm_sys.state == fsm.State.PREHEAT and tc_avg >= fsm_sys.target_pan_temp_c:
            #     fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TEMP_REACHED, {"tc_temp_c": tc})) # to SEAR1

            # if fsm_sys.state == fsm.State.SEAR1:
            #     time.sleep(5)
            #     fsm_sys.dispatch(fsm.EventMsg(fsm.Event.FLIP)) # to SEAR2

            # if fsm_sys.state == fsm.State.SEAR2:
            #     time.sleep(5)
            #     fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TARGET_MET)) # to REST

            # if fsm_sys.state == fsm.State.REST:
            #     time.sleep(5)
            #     fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TIMER_EXPIRED)) # to DONE

            # if fsm_sys.state == fsm.State.DONE:
            #     time.sleep(5)
            #     fsm_sys.dispatch(fsm.EventMsg(fsm.Event.RESET)) # to IDLE
            #     break


            # Decide "connectivity" from persistent fault flags
            # (fault_detection.apply_fault_flags(properties, flags) already ran above)
            probe_connected = (properties.fault_probe_dc != 1)
            tc_connected    = (properties.fault_tc_dc != 1)

            # Timer mode if either disconnect
            want_timer_mode = (not probe_connected) or (not tc_connected)

            if properties.warn_overtemp == 1 and prev_overtemp != 1:
                print("decrease knob rotation by 30 steps")
            prev_overtemp = properties.warn_overtemp

            if want_timer_mode and not timer_mode:
                timer_mode = True
                deadline_s = None
                deadline_label = ""
                print("Sensor disconnect -> switching to TIMER_MODE")
            elif (not want_timer_mode) and timer_mode:
                timer_mode = False
                deadline_s = None
                deadline_label = ""
                print("Sensors reconnected -> switching back to SENSOR_MODE")

            pan_c = tc_avg
            internal_c = temp_avg

            _status_line(pan_c, internal_c)

            if (not started) and fsm_sys.state == fsm.State.IDLE:
                fsm_sys.dispatch(fsm.EventMsg(fsm.Event.START))
                started = True

            fsm_sys.update_pan_temp(pan_c)

            # --- PREHEAT -> SEAR1 ---
            if fsm_sys.state == fsm.State.PREHEAT:

                # TIMER MODE (sensor disconnected)
                if timer_mode:

                    if deadline_s is None:
                        deadline_s = time.monotonic() + 10.0   # 3 min preheat
                        deadline_label = "PREHEAT"

                    rem = deadline_s - time.monotonic()
                    _countdown_line(deadline_label, rem)

                    if rem <= 0:
                        print()
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TEMP_REACHED))
                        deadline_s = None
                        deadline_label = ""

                # SENSOR MODE
                else:
                    if pan_c >= fsm_sys.target_pan_temp_c:
                        fsm_sys.dispatch(
                            fsm.EventMsg(fsm.Event.TEMP_REACHED, {"pan_c": pan_c})
                        )

            # --- SEAR1 -> SEAR2 ---
            if fsm_sys.state == fsm.State.SEAR1:
                if timer_mode:
                    if deadline_s is None:
                        deadline_s = time.monotonic() + SEAR_SIDE_S
                        deadline_label = "SEAR1"
                    rem = deadline_s - time.monotonic()
                    _countdown_line(deadline_label, rem)
                    if rem <= 0:
                        print()
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.FLIP))
                        deadline_s = None
                        deadline_label = ""
                else:
                    # SENSOR_MODE: auto-flip when internal hits a threshold
                    if (not flip_sent) and internal_c >= FLIP_PAN_C:
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.FLIP, {"pan_c": pan_c}))
                        flip_sent = True

            # --- SEAR2 -> REST ---
            if fsm_sys.state == fsm.State.SEAR2:
                if timer_mode:
                    if deadline_s is None:
                        deadline_s = time.monotonic() + SEAR_SIDE_S
                        deadline_label = "SEAR2"
                    rem = deadline_s - time.monotonic()
                    _countdown_line(deadline_label, rem)
                    if rem <= 0:
                        print()
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TARGET_MET))
                        deadline_s = None
                        deadline_label = ""
                else:
                    # SENSOR_MODE: internal target
                    if internal_c >= INTERNAL_TARGET_C:
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TARGET_MET, {"internal_c": internal_c}))

            # --- REST -> DONE ---
            if fsm_sys.state == fsm.State.REST:
                if timer_mode:
                    if deadline_s is None:
                        deadline_s = time.monotonic() + REST_S
                        deadline_label = "REST"
                    rem = deadline_s - time.monotonic()
                    _countdown_line(deadline_label, rem)
                    if rem <= 0:
                        print()
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TIMER_EXPIRED))
                        deadline_s = None
                        deadline_label = ""
                else:
                    # SENSOR_MODE
                    if deadline_s is None:
                        deadline_s = time.monotonic() + REST_S
                        deadline_label = "REST"
                    rem = deadline_s - time.monotonic()
                    _countdown_line(deadline_label, rem)
                    if rem <= 0:
                        print()
                        fsm_sys.dispatch(fsm.EventMsg(fsm.Event.TIMER_EXPIRED))
                        deadline_s = None
                        deadline_label = ""

            # --- DONE -> RESET -> IDLE (end test) ---
            if fsm_sys.state == fsm.State.DONE:
                fsm_sys.dispatch(fsm.EventMsg(fsm.Event.RESET))
                break



        except serial.SerialException as e:
            print(f"Serial error: {e}")
            try:
                ser_conn.close()
            except Exception:
                pass
            print("Reconnecting...")
            time.sleep(0.5)
            ser_conn = open_serial()

        except KeyboardInterrupt:
            try:
                # close connection
                ser_conn.close()

                # reset graph
                plot._t_buf.clear()
                plot._raw_buf.clear()
                plot._tc_buf.clear()
                _t0 = None
                plot._line_raw.set_data([], [])
                plot._line_tc.set_data([], [])

                # clear buffers for filters
                history.clear()
                med_adc = filter_and_derive.MedianFilter(MED_N)
                ma_adc = filter_and_derive.MovingAverage(MA_N)
                med_tc = filter_and_derive.MedianFilter(MED_N)
                ma_tc  = filter_and_derive.MovingAverage(MA_N)

                # clear linear regression and variance
                linreg.reset()
                var.reset()

                # clear fault detection
                fault_detection.FaultFlags.clear()

                # close all
                plt.close('all')
            except Exception:
                pass
            print("\nStopped.")
            break

if __name__ == "__main__":
    main()