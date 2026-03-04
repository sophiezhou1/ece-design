#test commit

import time
import serial
from serial.tools import list_ports
import math
import re
import numpy as np
from dataclasses import dataclass

import matplotlib.pyplot as plt
from collections import deque

import plot


@dataclass
class sensor_data:
    t_us: int
    adc: int
    tc: float

BAUD = 115200
TIMEOUT_S = 1.0

PORT_HINTS = [
    "/dev/cu.usbserial",   # common USB-serial adapters
    "/dev/cu.SLAB_USBtoUART",  # CP210x
    "/dev/cu.wchusbserial",    # CH34x
    "/dev/cu.usbmodem",        # some dev boards
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
        # split timestamp + raw first
        t_us_str, rest = line.split(",", 1)
        t_us = int(t_us_str.strip())

        # raw value
        adc_match = re.search(r"adc(\d+)", line)
        raw_match = re.search(r"raw=0x([0-9A-Fa-f]+)", rest)
        tc_match = re.search(r"tc=([-+]?\d*\.?\d+)", rest)
        fault_match = re.search(r"fault=(\d+)", rest)
        oc_match = re.search(r"OC=(\d+)", rest)
        scg_match = re.search(r"SCG=(\d+)", rest)
        scv_match = re.search(r"SCV=(\d+)", rest)

        if not all([raw_match, tc_match, fault_match, oc_match, scg_match, scv_match]):
            return None

        adc = int(adc_match.group(1))
        raw = int(raw_match.group(1), 16)
        tc = float(tc_match.group(1))
        fault = int(fault_match.group(1))
        fault_oc = int(oc_match.group(1))
        fault_scg = int(scg_match.group(1))
        fault_scv = int(scv_match.group(1))

        return (t_us,
                adc,
                raw,
                tc,
                fault,
                fault_oc,
                fault_scg,
                fault_scv)

    except Exception:
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
    return T

    # return c_to_f(T)
    # return vout



def main():
    ser_conn = open_serial()

    while True:
        try:
            line = ser_conn.readline()
            if line == b"":
                continue

            text = line.decode(errors="ignore").strip()
            if not text:
                continue

            (t_us, adc, _, tc, fault, fault_oc, fault_scg, fault_scv) = parse_line(text)
            temp = thermistor_conv(adc)

            # [1] non-filtered outputs
            print(f"{temp:.2f} {tc:.2f}")
            plot.update_plot(t_us, temp, tc)

            # data = sensor_data(t_us, temp, tc) # adc is preprocessed temp


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
                ser_conn.close()
                plot._t_buf.clear()
                plot._raw_buf.clear()
                plot._tc_buf.clear()

                _t0 = None

                plot._line_raw.set_data([], [])
                plot._line_tc.set_data([], [])

                plt.close('all')
            except Exception:
                pass
            print("\nStopped.")
            break

if __name__ == "__main__":
    main()