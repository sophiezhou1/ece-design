#!/usr/bin/env python3
"""
Barebones BLE reader for sensor_set payload.
Requires: bleak (pip install bleak)
"""

import asyncio
import struct
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "sensor_set"
CHAR_UUID = "78563412-9abc-def0-1234-56780100ee01"

# < = little-endian
# I   uint32 seq
# Q   uint64 timestamp_us
# f   float thermocouple_c
# f   float food_probe_c
# B   uint8 control_flags
# B   uint8 tc_fault_flags
# 2x  padding
PACK_FMT = "<IQffBB2x"
PACK_LEN = struct.calcsize(PACK_FMT)


def decode_packet(data: bytearray):
    if len(data) < PACK_LEN:
        return None
    seq, ts_us, tc, fp, ctrl, faults = struct.unpack(PACK_FMT, data[:PACK_LEN])
    return {
        "seq": seq,
        "timestamp_us": ts_us,
        "thermocouple_c": tc,
        "food_probe_c": fp,
        "control_flags": ctrl,
        "tc_fault_flags": faults,
    }


async def main():
    print("Scanning for device...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, _: (d.name or "") == DEVICE_NAME,
        timeout=15.0,
    )
    if not dev:
        raise RuntimeError(f"Could not find BLE device named '{DEVICE_NAME}'")

    async with BleakClient(dev) as client:
        print(f"Connected: {dev.address}")

        def on_notify(_handle: int, data: bytearray):
            pkt = decode_packet(data)
            if pkt:
                print(pkt)

        await client.start_notify(CHAR_UUID, on_notify)
        print("Subscribed to notifications. Listening for 60 minutes...")
        await asyncio.sleep(60 * 60)
        await client.stop_notify(CHAR_UUID)


if __name__ == "__main__":
    asyncio.run(main())
