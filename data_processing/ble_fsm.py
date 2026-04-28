import argparse
import asyncio
import math
import struct
import time
import sys
import contextlib
from collections import deque
from dataclasses import dataclass, asdict, field
from typing import Optional

from bleak import BleakClient, BleakScanner

import filter_and_derive
import fault_detection
import fsm

# BLE UUIDs as shown by macOS/CoreBluetooth
SERVICE_UUID = "01ee0000-7856-3412-f0de-bc9a12345678"
SERVICE_UUID_ALT = "78563412-9abc-def0-1234-56780000ee01"
CHAR_UUID = "01ee0001-7856-3412-f0de-bc9a12345678"

DEFAULT_NAME = "sensor_set"
PACK_FMT = "<IQffBB2x"
PACK_LEN = struct.calcsize(PACK_FMT)

# buffer
HISTORY_MAX = 5000

# filtering config
MA_N = 10
MED_N = 3


def _fmt_mmss(sec: float) -> str:
    sec_i = max(0, int(sec + 0.5))
    m = sec_i // 60
    s = sec_i % 60
    return f"{m:02d}:{s:02d}"


def _render_live_line(state: "TelemetryState") -> str:
    if not state.latest_sample:
        return "waiting for samples..."

    pan_c = state.latest_sample.tc
    internal_c = state.latest_sample.adc
    faults = ",".join(state.fault_strings) if state.fault_strings else "none"
    timer_txt = (
        f"{state.timer_label} {_fmt_mmss(state.timer_remaining_s)}"
        if state.timer_mode and state.timer_label
        else ("ON" if state.timer_mode else "OFF")
    )

    return (
        f"pan={pan_c:5.2f}C "
        f"internal={internal_c:5.2f}C "
        f"fsm={state.fsm_state} "
        f"timer={timer_txt} "
        f"faults={faults}"
    )


def _print_live_line(state: "TelemetryState"):
    msg = _render_live_line(state)
    sys.stdout.write("\r" + msg + " " * 12)
    sys.stdout.flush()


def _freeze_line():
    sys.stdout.write("\n")
    sys.stdout.flush()


# Helper to silence FSM prints
@contextlib.contextmanager
def _silence_fsm_prints():
    saved_stdout = sys.stdout
    try:
        with open("/dev/null", "w") as devnull:
            sys.stdout = devnull
            yield
    finally:
        sys.stdout = saved_stdout

@dataclass
class BlePacket:
    seq: int
    t_us: int
    tc_c: float
    food_probe_raw: float
    control_flags: int
    tc_fault_flags: int


@dataclass
class SensorData:
    t_us: int
    adc: float
    tc: float


@dataclass
class Properties:
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


@dataclass
class TelemetryState:
    latest_packet: Optional[BlePacket] = None
    latest_sample: Optional[SensorData] = None
    properties: Properties = field(default_factory=Properties)
    fault_strings: list[str] = field(default_factory=list)
    fault_codes: list[int] = field(default_factory=list)
    event_tags: list[str] = field(default_factory=list)
    fsm_state: str | None = None
    timer_mode: bool = False
    timer_label: str = ""
    timer_remaining_s: float = 0.0

    def as_dict(self) -> dict:
        return {
            "latest_packet": asdict(self.latest_packet) if self.latest_packet else None,
            "latest_sample": asdict(self.latest_sample) if self.latest_sample else None,
            "properties": asdict(self.properties),
            "fault_strings": list(self.fault_strings),
            "fault_codes": list(self.fault_codes),
            "event_tags": list(self.event_tags),
            "fsm_state": self.fsm_state,
            "timer_mode": self.timer_mode,
            "timer_label": self.timer_label,
            "timer_remaining_s": self.timer_remaining_s,
        }


class Pipeline:
    FAULT_CODE_MAP = {
        "PROBE_DISCONNECT": 1,
        "THERMOCOUPLE_DISCONNECT": 2,
        "OVERTEMP": 3,
        "THERMISTOR_ADC_RAIL": 4,
        "TC_OPEN": 5,
        "TC_SHORT_GND": 6,
        "TC_SHORT_VCC": 7,
    }

    def __init__(self):
        self.history = deque(maxlen=HISTORY_MAX)

        self.med_adc = filter_and_derive.MedianFilter(MED_N)
        self.ma_adc = filter_and_derive.MovingAverage(MA_N)
        self.med_tc = filter_and_derive.MedianFilter(MED_N)
        self.ma_tc = filter_and_derive.MovingAverage(MA_N)

        self.linreg = filter_and_derive.MovingLinearRegression(50, 50)
        self.var = filter_and_derive.MovingVariance(50, 50)

        cfg = fault_detection.ThresholdConfig(overtemp_set=40, overtemp_clr=38, overtemp_trip_time=0.5)
        self.monitor = fault_detection.FaultMonitor(cfg)

        self.state = TelemetryState()
        self.started = False
        self.flip_sent = False
        self.cycle_complete = False
        self.prev_overtemp = 0
        self.timer_mode = False
        self.deadline_s: float | None = None
        self.deadline_label = ""
        self.preheat_target_c = 35.0
        self.flip_pan_c = 40.0
        self.internal_target_c = 50.0
        self.sear_side_s = 10.0
        self.rest_s = 10.0
        self.fsm_sys = fsm.FSM(target_pan_temp_c=self.preheat_target_c)

    def _emit_tag(self, tags: list[str], name: str) -> None:
        tags.append(name)

    def _dispatch_with_tag(self, tags: list[str], event: fsm.Event, payload: dict | None = None):
        payload = payload or {}
        prev_state = self.fsm_sys.state
        with _silence_fsm_prints():
            ok = self.fsm_sys.dispatch(fsm.EventMsg(event, payload))
        if ok:
            self._emit_tag(tags, f"FSM_EVENT:{event.name}")
            if self.fsm_sys.state != prev_state:
                self._emit_tag(tags, f"FSM_STATE:{prev_state.name}->{self.fsm_sys.state.name}")
        return ok

    @staticmethod
    def thermistor_conv(raw: float) -> float:
        ADC_MAX = 4095
        rfixed = 100000
        vcc = 3.295

        adc_raw = max(1, min(ADC_MAX - 1, int(raw)))
        vout = adc_raw * vcc / ADC_MAX

        A = -0.1943 * 10 ** -3
        B = 3.4023 * 10 ** -4
        C = -2.3843 * 10 ** -7

        rth = vout * rfixed / (vcc - vout)
        Tinv = A + B * math.log(rth) + C * (math.log(rth)) ** 3
        T = Tinv ** -1
        return T - 273.15

    @staticmethod
    def decode_packet(data: bytearray) -> Optional[BlePacket]:
        if len(data) < PACK_LEN:
            return None
        seq, ts_us, tc, probe_raw, ctrl, tc_faults = struct.unpack(PACK_FMT, data[:PACK_LEN])
        return BlePacket(seq, ts_us, tc, probe_raw, ctrl, tc_faults)

    def process_packet(self, pkt: BlePacket) -> TelemetryState:
        event_tags: list[str] = []
        # raw probe from BLE payload -> temp conversion
        adc_raw_int = int(pkt.food_probe_raw)
        temp = pkt.food_probe_raw

        sample = SensorData(pkt.t_us, temp, pkt.tc_c)
        self.history.append(sample)

        temp_med = self.med_adc.update(temp)
        temp_avg = self.ma_adc.update(temp_med)
        tc_med = self.med_tc.update(pkt.tc_c)
        tc_avg = self.ma_tc.update(tc_med)

        slope = self.linreg.update(pkt.t_us * 1e-6, temp_avg)
        if slope is not None:
            self.state.properties.slope = slope * 60

        variance = self.var.update(temp_avg)
        if variance is not None:
            self.state.properties.var = variance

        fault = 1 if (pkt.control_flags & 0x01) else 0
        fault_oc = pkt.tc_fault_flags & 0x01
        fault_scg = (pkt.tc_fault_flags >> 1) & 0x01
        fault_scv = (pkt.tc_fault_flags >> 2) & 0x01

        # intern currently not transmitted over BLE; keep None-equivalent using NaN
        flags = self.monitor.update(pkt.t_us, adc_raw_int, pkt.tc_c, math.nan,
                                    fault, fault_oc, fault_scg, fault_scv)
        fault_detection.apply_fault_flags(self.state.properties, flags)

        fault_strings: list[str] = []
        if adc_raw_int >= 4095 or adc_raw_int <= 0:
            fault_strings.append("THERMISTOR_ADC_RAIL")
        if self.state.properties.fault_probe_dc == 1:
            fault_strings.append("PROBE_DISCONNECT")
        if self.state.properties.fault_tc_dc == 1:
            fault_strings.append("THERMOCOUPLE_DISCONNECT")
        if self.state.properties.warn_overtemp == 1:
            fault_strings.append("OVERTEMP")
            if self.prev_overtemp != 1:
                self._emit_tag(event_tags, "ACTION:DECREASE_KNOB_125")
        self.prev_overtemp = self.state.properties.warn_overtemp
        if fault_oc == 1:
            fault_strings.append("TC_OPEN")
        if fault_scg == 1:
            fault_strings.append("TC_SHORT_GND")
        if fault_scv == 1:
            fault_strings.append("TC_SHORT_VCC")

        # stable order + dedupe
        fault_strings = list(dict.fromkeys(fault_strings))
        self.state.fault_strings = fault_strings
        self.state.fault_codes = [self.FAULT_CODE_MAP[name] for name in fault_strings]

        # ===== FSM / TIMER MODE FLOW =====
        probe_connected = (self.state.properties.fault_probe_dc != 1)
        tc_connected = (self.state.properties.fault_tc_dc != 1)
        want_timer_mode = (not probe_connected) or (not tc_connected)

        if want_timer_mode and not self.timer_mode:
            self.timer_mode = True
            self.deadline_s = None
            self.deadline_label = ""
            if not probe_connected:
                self._emit_tag(event_tags, "SENSOR:PROBE_DISCONNECTED")
            if not tc_connected:
                self._emit_tag(event_tags, "SENSOR:TC_DISCONNECTED")
            self._emit_tag(event_tags, "MODE:TIMER_ON")
        elif (not want_timer_mode) and self.timer_mode:
            self.timer_mode = False
            self.deadline_s = None
            self.deadline_label = ""
            self._emit_tag(event_tags, "SENSOR:RECONNECTED")
            self._emit_tag(event_tags, "MODE:TIMER_OFF")

        pan_c = tc_avg
        internal_c = temp_avg

        if (not self.started) and self.fsm_sys.state == fsm.State.IDLE:
            self._dispatch_with_tag(event_tags, fsm.Event.START)
            self.started = True

        self.fsm_sys.update_pan_temp(pan_c)

        now = time.monotonic()
        # PREHEAT -> SEAR1
        if self.fsm_sys.state == fsm.State.PREHEAT:
            if self.timer_mode:
                if self.deadline_s is None:
                    self.deadline_s = now + 10.0
                    self.deadline_label = "PREHEAT"
                    self._emit_tag(event_tags, "TIMER:PREHEAT_START")
                rem = self.deadline_s - now
                if rem <= 0:
                    self._dispatch_with_tag(event_tags, fsm.Event.TEMP_REACHED)
                    self.deadline_s = None
                    self.deadline_label = ""
                    self._emit_tag(event_tags, "TIMER:PREHEAT_DONE")
            else:
                if pan_c >= self.fsm_sys.target_pan_temp_c:
                    self._dispatch_with_tag(event_tags, fsm.Event.TEMP_REACHED, {"pan_c": pan_c})

        # SEAR1 -> SEAR2
        if self.fsm_sys.state == fsm.State.SEAR1:
            if self.timer_mode:
                if self.deadline_s is None:
                    self.deadline_s = now + self.sear_side_s
                    self.deadline_label = "SEAR1"
                    self._emit_tag(event_tags, "TIMER:SEAR1_START")
                rem = self.deadline_s - now
                if rem <= 0:
                    self._dispatch_with_tag(event_tags, fsm.Event.FLIP)
                    self.deadline_s = None
                    self.deadline_label = ""
                    self._emit_tag(event_tags, "TIMER:SEAR1_DONE")
            else:
                if (not self.flip_sent) and internal_c >= self.flip_pan_c:
                    self._dispatch_with_tag(event_tags, fsm.Event.FLIP, {"pan_c": pan_c})
                    self.flip_sent = True

        # SEAR2 -> REST
        if self.fsm_sys.state == fsm.State.SEAR2:
            if self.timer_mode:
                if self.deadline_s is None:
                    self.deadline_s = now + self.sear_side_s
                    self.deadline_label = "SEAR2"
                    self._emit_tag(event_tags, "TIMER:SEAR2_START")
                rem = self.deadline_s - now
                if rem <= 0:
                    self._dispatch_with_tag(event_tags, fsm.Event.TARGET_MET)
                    self.deadline_s = None
                    self.deadline_label = ""
                    self._emit_tag(event_tags, "TIMER:SEAR2_DONE")
            else:
                if internal_c >= self.internal_target_c:
                    self._dispatch_with_tag(event_tags, fsm.Event.TARGET_MET, {"internal_c": internal_c})

        # REST -> DONE
        if self.fsm_sys.state == fsm.State.REST:
            if self.deadline_s is None:
                self.deadline_s = now + self.rest_s
                self.deadline_label = "REST"
                self._emit_tag(event_tags, "TIMER:REST_START")
            rem = self.deadline_s - now
            if rem <= 0:
                self._dispatch_with_tag(event_tags, fsm.Event.TIMER_EXPIRED)
                self.deadline_s = None
                self.deadline_label = ""
                self._emit_tag(event_tags, "TIMER:REST_DONE")

        if self.fsm_sys.state == fsm.State.DONE and not self.cycle_complete:
            self._emit_tag(event_tags, "FSM:CYCLE_COMPLETE")
            self.cycle_complete = True

        self.state.latest_packet = pkt
        self.state.latest_sample = sample
        self.state.event_tags = list(dict.fromkeys(event_tags))
        self.state.fsm_state = self.fsm_sys.state.name
        self.state.timer_mode = self.timer_mode
        self.state.timer_label = self.deadline_label
        self.state.timer_remaining_s = max(0.0, (self.deadline_s - now) if self.deadline_s else 0.0)
        return self.state

    def reset(self):
        self.history.clear()
        self.med_adc = filter_and_derive.MedianFilter(MED_N)
        self.ma_adc = filter_and_derive.MovingAverage(MA_N)
        self.med_tc = filter_and_derive.MedianFilter(MED_N)
        self.ma_tc = filter_and_derive.MovingAverage(MA_N)
        self.linreg.reset()
        self.var.reset()
        self.state = TelemetryState()
        self.started = False
        self.flip_sent = False
        self.cycle_complete = False
        self.prev_overtemp = 0
        self.timer_mode = False
        self.deadline_s = None
        self.deadline_label = ""


def _candidate_matcher(name: Optional[str], address: Optional[str]):
    valid_service_uuids = {SERVICE_UUID.lower(), SERVICE_UUID_ALT.lower()}

    def _match(device, adv_data):
        if address and device.address.lower() == address.lower():
            return True
        if adv_data and adv_data.service_uuids:
            advertised = {u.lower() for u in adv_data.service_uuids}
            if advertised.intersection(valid_service_uuids):
                return True
        if name:
            return (device.name or "") == name
        return False

    return _match


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default=DEFAULT_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-seconds", type=float, default=20.0)
    parser.add_argument("--duration-seconds", type=float, default=60 * 60)
    args = parser.parse_args()

    print(f"Scanning for BLE source for {args.scan_seconds:.0f}s...")
    dev = await BleakScanner.find_device_by_filter(
        _candidate_matcher(args.name, args.address), timeout=args.scan_seconds
    )

    if not dev:
        print("No exact match found. Nearby devices:")
        devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
        for _d, (dev_info, adv) in devices.items():
            uuids = list((adv.service_uuids or []))
            print(f"  - {dev_info.address} | name={dev_info.name!r} | uuids={uuids}")
        raise RuntimeError("Could not find BLE source. Try --address <id> from list above.")

    pipeline = Pipeline()
    queue: asyncio.Queue[bytearray] = asyncio.Queue(maxsize=200)

    async with BleakClient(dev) as client:
        print(f"Connected: {dev.address}")

        def on_notify(_handle: int, data: bytearray):
            if queue.full():
                _ = queue.get_nowait()
            queue.put_nowait(data)

        await client.start_notify(CHAR_UUID, on_notify)
        print("Subscribed. Processing packets...")

        end_time = asyncio.get_event_loop().time() + args.duration_seconds
        try:
            while asyncio.get_event_loop().time() < end_time:
                data = await queue.get()
                pkt = pipeline.decode_packet(data)
                if not pkt:
                    continue
                state = pipeline.process_packet(pkt)

                transition_tags = [t for t in state.event_tags if t.startswith("FSM_STATE:")]
                mode_tags = [t for t in state.event_tags if t.startswith("MODE:")]
                action_tags = [t for t in state.event_tags if t.startswith("ACTION:")]
                sensor_tags = [t for t in state.event_tags if t.startswith("SENSOR:")]

                _print_live_line(state)

                if transition_tags or mode_tags or action_tags or sensor_tags:
                    _freeze_line()
                    for tag in transition_tags:
                        print(f"  {tag.split(':', 1)[1]}")
                    for tag in mode_tags:
                        print(f"  {tag.split(':', 1)[1]}")
                    for tag in action_tags:
                        print(f"  {tag.split(':', 1)[1]}")
                    for tag in sensor_tags:
                        print(f"  {tag.split(':', 1)[1]}")

                if pipeline.cycle_complete:
                    _freeze_line()
                    print("Cook cycle complete.")
                    break
        finally:
            _freeze_line()
            await client.stop_notify(CHAR_UUID)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
