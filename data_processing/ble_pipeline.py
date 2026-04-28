#!/usr/bin/env python3
"""BLE reader with app-oriented data structures and processing pipeline.

This mirrors the old serial processing flow but consumes BLE notifications.
"""

import argparse
import asyncio
import math
import struct
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

    def as_dict(self) -> dict:
        return {
            "latest_packet": asdict(self.latest_packet) if self.latest_packet else None,
            "latest_sample": asdict(self.latest_sample) if self.latest_sample else None,
            "properties": asdict(self.properties),
            "fault_strings": list(self.fault_strings),
            "fault_codes": list(self.fault_codes),
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

        cfg = fault_detection.ThresholdConfig(overtemp_set=100, overtemp_clr=98, overtemp_trip_time=0.5)
        self.monitor = fault_detection.FaultMonitor(cfg)
        self.fsm_sys = fsm.FSM(target_pan_temp_c=30)

        self.state = TelemetryState()

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
        # raw probe from BLE payload -> temp conversion
        adc_raw_int = int(pkt.food_probe_raw)
        # temp = self.thermistor_conv(pkt.food_probe_raw)
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
        tc_disconnect = 1 if (pkt.control_flags & 0x08) else 0  
        fault_oc = pkt.tc_fault_flags & 0x01
        fault_scg = (pkt.tc_fault_flags >> 1) & 0x01
        fault_scv = (pkt.tc_fault_flags >> 2) & 0x01

        # intern currently not transmitted over BLE; keep None-equivalent using NaN
        internal_c_for_fault = -1.0 if tc_disconnect else math.nan
        flags = self.monitor.update(pkt.t_us, adc_raw_int, pkt.tc_c, internal_c_for_fault,
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
        if self.state.properties.fault_oc == 1:
            fault_strings.append("TC_OPEN")
        if self.state.properties.fault_scg == 1:
            fault_strings.append("TC_SHORT_GND")
        if self.state.properties.fault_scv == 1:
            fault_strings.append("TC_SHORT_VCC")

        # stable order + dedupe
        fault_strings = list(dict.fromkeys(fault_strings))
        self.state.fault_strings = fault_strings
        self.state.fault_codes = [self.FAULT_CODE_MAP[name] for name in fault_strings]

        self.state.latest_packet = pkt
        self.state.latest_sample = sample
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

                # predictable, stable output for downstream app integration
                out = state.as_dict()
                if out["latest_sample"] is not None:
                    print(
                        f"{out['latest_sample']['adc']:.2f} {out['latest_sample']['tc']:.2f} "
                        f"fault={out['properties']['fault']} "
                        f"fault_strings={out['fault_strings']} "
                        f"fault_codes={out['fault_codes']}"
                    )
        finally:
            await client.stop_notify(CHAR_UUID)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
