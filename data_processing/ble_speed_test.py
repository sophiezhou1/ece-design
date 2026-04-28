#!/usr/bin/env python3
"""BLE notification speed tester for sensor_set.

This is only for measuring Bluetooth notification throughput and packet loss.
It matches the packet format used by ble_pipeline.py and connects directly to
macOS/CoreBluetooth's chosen device ID from the current scan output.
"""

import argparse
import asyncio
import struct
import time
from dataclasses import dataclass
from typing import Optional

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "sensor_set"
DEVICE_ADDRESS = "BD6E8DEF-CC0E-654E-AC7A-BA15BC11E4A0"

# UUIDs must match ble_pipeline.py / ble.c
SERVICE_UUID = "01ee0000-7856-3412-f0de-bc9a12345678"
SERVICE_UUID_ALT = "78563412-9abc-def0-1234-56780000ee01"
CHAR_UUID = "01ee0001-7856-3412-f0de-bc9a12345678"

PACK_FMT = "<IQffBB2x"
PACK_LEN = struct.calcsize(PACK_FMT)


@dataclass
class Stats:
    total_bytes: int = 0
    total_packets: int = 0
    lost_packets: int = 0
    short_packets: int = 0
    duplicate_or_reordered: int = 0
    last_seq: Optional[int] = None
    first_seq: Optional[int] = None
    latest_seq: Optional[int] = None


def decode_seq(data: bytearray) -> Optional[int]:
    if len(data) < 4:
        return None
    return struct.unpack_from("<I", data, 0)[0]


def decode_packet_preview(data: bytearray) -> str:
    if len(data) < PACK_LEN:
        return f"short packet len={len(data)}"

    seq, ts_us, tc_c, food_probe_c, ctrl, tc_faults = struct.unpack(PACK_FMT, data[:PACK_LEN])
    return (
        f"seq={seq} t_us={ts_us} pan={tc_c:.2f}C meat={food_probe_c:.2f}C "
        f"ctrl=0x{ctrl:02X} tc_faults=0x{tc_faults:02X}"
    )


def make_matcher(name: str, address: Optional[str]):
    valid_services = {SERVICE_UUID.lower(), SERVICE_UUID_ALT.lower()}

    def matcher(device, adv_data):
        if address and device.address.lower() == address.lower():
            return True

        if adv_data and adv_data.service_uuids:
            advertised = {uuid.lower() for uuid in adv_data.service_uuids}
            if advertised.intersection(valid_services):
                return True

        return (device.name or "") == name

    return matcher


async def find_device(name: str, address: Optional[str], scan_seconds: float):
    print(f"Scanning for {name!r} for {scan_seconds:.1f}s...")

    dev = await BleakScanner.find_device_by_filter(
        make_matcher(name, address),
        timeout=scan_seconds,
    )

    if dev:
        return dev

    print("\nNo exact match found. Nearby devices:")
    devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
    for _, (dev_info, adv) in devices.items():
        uuids = list(adv.service_uuids or [])
        print(f"  - name={dev_info.name!r}, address={dev_info.address}, uuids={uuids}")

    return None


async def main():
    parser = argparse.ArgumentParser(description="Measure BLE notification speed from sensor_set.")
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=DEVICE_ADDRESS)
    parser.add_argument("--char", default=CHAR_UUID)
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--duration-seconds", type=float, default=60.0)
    parser.add_argument("--interval-seconds", type=float, default=1.0)
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args()

    stats = Stats()

    dev = await find_device(args.name, args.address, args.scan_seconds)
    if dev is None:
        print("\nDevice not found. If macOS changed the CoreBluetooth address, rerun without --address.")
        return

    print(f"\nConnecting to name={dev.name!r}, address={dev.address}...")

    async with BleakClient(dev, timeout=30.0) as client:
        print("Connected:", client.is_connected)

        try:
            mtu = client.mtu_size
            print(f"Client MTU reported by Bleak: {mtu}")
        except Exception:
            print("Client MTU not available from this Bleak backend.")

        print("\nServices:")
        for service in client.services:
            print("Service:", service.uuid)
            for char in service.characteristics:
                marker = "  <-- target" if char.uuid.lower() == args.char.lower() else ""
                print(f"  Characteristic: {char.uuid} {char.properties}{marker}")

        first_notification_printed = False

        def notification_handler(sender, data: bytearray):
            nonlocal first_notification_printed

            stats.total_bytes += len(data)
            stats.total_packets += 1

            if len(data) < PACK_LEN:
                stats.short_packets += 1

            seq = decode_seq(data)
            if seq is None:
                return

            if stats.first_seq is None:
                stats.first_seq = seq

            if stats.last_seq is not None:
                if seq > stats.last_seq + 1:
                    stats.lost_packets += seq - (stats.last_seq + 1)
                elif seq <= stats.last_seq:
                    stats.duplicate_or_reordered += 1

            stats.last_seq = seq
            stats.latest_seq = seq

            if not args.no_preview and not first_notification_printed:
                print("\nFirst packet:", decode_packet_preview(data))
                first_notification_printed = True

        print(f"\nStarting notifications on {args.char}")
        await client.start_notify(args.char, notification_handler)
        print("Receiving data. Press Ctrl+C to stop.\n")

        start_time = time.perf_counter()
        prev_time = start_time
        prev_bytes = 0
        prev_packets = 0
        prev_lost = 0
        prev_short = 0
        prev_dup_reorder = 0

        try:
            while True:
                await asyncio.sleep(args.interval_seconds)

                now = time.perf_counter()
                elapsed = now - prev_time
                total_elapsed = now - start_time

                bytes_diff = stats.total_bytes - prev_bytes
                packets_diff = stats.total_packets - prev_packets
                lost_diff = stats.lost_packets - prev_lost
                short_diff = stats.short_packets - prev_short
                dup_reorder_diff = stats.duplicate_or_reordered - prev_dup_reorder

                bitrate_bps = (bytes_diff * 8.0) / elapsed if elapsed > 0 else 0.0
                bitrate_mbps = bitrate_bps / 1_000_000.0
                bytes_per_sec = bytes_diff / elapsed if elapsed > 0 else 0.0
                packets_per_sec = packets_diff / elapsed if elapsed > 0 else 0.0
                avg_packet_size = (bytes_diff / packets_diff) if packets_diff else 0.0

                overall_mbps = (stats.total_bytes * 8.0) / total_elapsed / 1_000_000.0 if total_elapsed > 0 else 0.0

                print(
                    f"Rate: {bitrate_mbps:.4f} Mbps | "
                    f"{bytes_per_sec:.1f} B/s | "
                    f"{packets_per_sec:.1f} pkts/s | "
                    f"avg {avg_packet_size:.1f} B/pkt | "
                    f"lost {lost_diff} | "
                    f"short {short_diff} | "
                    f"dup/reorder {dup_reorder_diff} | "
                    f"seq {stats.latest_seq} | "
                    f"overall {overall_mbps:.4f} Mbps"
                )

                prev_time = now
                prev_bytes = stats.total_bytes
                prev_packets = stats.total_packets
                prev_lost = stats.lost_packets
                prev_short = stats.short_packets
                prev_dup_reorder = stats.duplicate_or_reordered

                if args.duration_seconds > 0 and total_elapsed >= args.duration_seconds:
                    break

        finally:
            await client.stop_notify(args.char)
            total_elapsed = max(time.perf_counter() - start_time, 1e-9)
            print("\nNotifications stopped.")
            print("\nSummary:")
            print(f"  Total time: {total_elapsed:.2f}s")
            print(f"  Total bytes: {stats.total_bytes}")
            print(f"  Total packets: {stats.total_packets}")
            print(f"  Average rate: {(stats.total_bytes * 8.0) / total_elapsed / 1_000_000.0:.4f} Mbps")
            print(f"  Average packet rate: {stats.total_packets / total_elapsed:.2f} pkts/s")
            print(f"  Lost packets from seq gaps: {stats.lost_packets}")
            print(f"  Duplicate/reordered packets: {stats.duplicate_or_reordered}")
            print(f"  Short packets: {stats.short_packets}")
            print(f"  First seq: {stats.first_seq}")
            print(f"  Last seq: {stats.last_seq}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped by user.")