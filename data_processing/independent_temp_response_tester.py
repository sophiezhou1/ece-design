import math
import queue
import re
import threading
import time

import serial
from serial.tools import list_ports

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
        t_us_str, rest = line.split(",", 1)
        t_us = int(t_us_str.strip())

        adc_match = re.search(r"adc(\d+)", line)
        tc_match = re.search(r"tc=([-+]?\d*\.?\d+)", rest)

        if not all([adc_match, tc_match]):
            return None

        adc = int(adc_match.group(1))
        tc = float(tc_match.group(1))
        return t_us, adc, tc

    except Exception:
        return None


def thermistor_conv(raw: int) -> float:
    ADC_MAX = 4095
    rfixed = 100000
    vcc = 3.295

    raw = max(1, min(ADC_MAX - 1, int(raw)))
    vout = raw * vcc / ADC_MAX

    A = -0.1943e-3
    B = 3.4023e-4
    C = -2.3843e-7

    rth = vout * rfixed / (vcc - vout)
    Tinv = A + B * math.log(rth) + C * (math.log(rth)) ** 3
    T = 1.0 / Tinv
    return T - 273.15


class StepResponseTester:
    def __init__(self, delta_c: float = 1.0):
        self.delta_c = delta_c
        self.waiting = True
        self.start_temp = None
        self.start_time = None
        self.results = []

    def trigger(self, current_temp: float, current_time_s: float):
        self.start_temp = current_temp
        self.start_time = current_time_s
        self.waiting = False
        print(f"\n[START] Baseline temp: {current_temp:.2f} C")
        print(f"Waiting for +{self.delta_c:.2f} C increase...")

    def update(self, current_temp: float, current_time_s: float):
        if self.waiting:
            return False

        delta = current_temp - self.start_temp
        if delta >= self.delta_c:
            dt = current_time_s - self.start_time
            self.results.append(dt)
            print(
                f"[DONE] Temp rose by {delta:.2f} C "
                f"from {self.start_temp:.2f} C to {current_temp:.2f} C "
                f"in {dt:.3f} s\n"
            )
            self.waiting = True
            self.start_temp = None
            self.start_time = None
            return True

        return False

    def print_summary(self):
        print("\n=== RESPONSE TIMES ===")
        if not self.results:
            print("No completed runs.")
        else:
            for i, t_s in enumerate(self.results, start=1):
                print(f"Run {i}: {t_s:.3f} s")
        print("======================\n")


def input_worker(cmd_queue: queue.Queue, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            user_input = input().strip().lower()
            if user_input != "":
                cmd_queue.put(user_input)
        except EOFError:
            break
        except Exception:
            break


def main():
    ser_conn = open_serial()
    tester = StepResponseTester(delta_c=1.0)

    cmd_queue = queue.Queue()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=input_worker,
        args=(cmd_queue, stop_event),
        daemon=True,
    )
    thread.start()

    print("Type s then ENTER to start a timing run.")
    print("Type q then ENTER to quit.")

    last_print_time = 0.0

    try:
        while True:
            line = ser_conn.readline()
            if line == b"":
                continue

            text = line.decode(errors="ignore").strip()
            if not text:
                continue

            parsed = parse_line(text)
            if parsed is None:
                continue

            t_us, adc, tc = parsed
            temp_c = thermistor_conv(adc)
            current_time_s = t_us * 1e-6

            if time.time() - last_print_time > 0.5:
                status = "WAITING" if tester.waiting else "TIMING"
                print(f"[{status}] Probe: {temp_c:.2f} C | TC: {tc:.2f} C")
                last_print_time = time.time()

            while not cmd_queue.empty():
                cmd = cmd_queue.get()

                if cmd == "q":
                    return

                if cmd == "s":
                    if tester.waiting:
                        tester.trigger(temp_c, current_time_s)
                    else:
                        print("Timer already running. Wait for current run to finish.")
                else:
                    print("Unknown command. Use s to start or q to quit.")

            finished = tester.update(temp_c, current_time_s)
            if finished:
                print("Type s then ENTER to start the next run. Type q then ENTER to quit.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        stop_event.set()
        try:
            ser_conn.close()
        except Exception:
            pass
        tester.print_summary()


if __name__ == "__main__":
    main()
