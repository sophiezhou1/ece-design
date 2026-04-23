#!/usr/bin/env python3
"""Barebones BLE reader for sensor_set payload.

Requires: bleak (pip install bleak)
"""

import asyncio
import struct
import argparse
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "sensor_set"
# NimBLE BLE_UUID128_INIT(...) byte-order as shown by host scanners (macOS/CoreBluetooth).
SERVICE_UUID = "01ee0000-7856-3412-f0de-bc9a12345678"
CHAR_UUID = "01ee0001-7856-3412-f0de-bc9a12345678"
# Canonical human-order form (accepted as alternate match to reduce confusion).
SERVICE_UUID_ALT = "78563412-9abc-def0-1234-56780000ee01"

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
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default=DEVICE_NAME, help="BLE local name (optional)")
    parser.add_argument("--address", default=None, help="MAC/UUID (strongest match)")
    parser.add_argument("--scan-seconds", type=float, default=20.0)
    args = parser.parse_args()

    print(f"Scanning for device for {args.scan_seconds:.0f}s...")
    valid_service_uuids = {SERVICE_UUID.lower(), SERVICE_UUID_ALT.lower()}

    def _matches(device, adv_data):
        if args.address and device.address.lower() == args.address.lower():
            return True
        if adv_data and adv_data.service_uuids:
            advertised = {u.lower() for u in adv_data.service_uuids}
            if advertised.intersection(valid_service_uuids):
                return True
        if args.name:
            return (device.name or "") == args.name
        return False

    dev = await BleakScanner.find_device_by_filter(_matches, timeout=args.scan_seconds)
    if not dev:
        print("No exact match found. Nearby devices:")
        devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
        for d, (dev_info, adv) in devices.items():
            uuids = list((adv.service_uuids or []))
            print(f"  - {dev_info.address} | name={dev_info.name!r} | uuids={uuids}")
        raise RuntimeError("Could not find device. Try --address <id> from the list above.")

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
