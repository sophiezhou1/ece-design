import asyncio
import struct
import time
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "ESP32_BLE_TPUT"
CHAR_UUID = "90ab90ab-90ab-90ab-90ab-90ab90abcdef"

total_bytes = 0
total_packets = 0
lost_packets = 0
last_seq = None


def notification_handler(sender, data: bytearray):
    global total_bytes, total_packets, lost_packets, last_seq

    total_bytes += len(data)
    total_packets += 1

    if len(data) >= 4:
        seq = struct.unpack_from("<I", data, 0)[0]
        if last_seq is not None and seq > last_seq + 1:
            lost_packets += seq - (last_seq + 1)
        last_seq = seq


async def main():
    global total_bytes, total_packets, lost_packets

    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = None
    for d in devices:
        print(f"name={d.name}, address={d.address}")
        if d.name == DEVICE_NAME:
            target = d

    if target is None:
        print("Device not found")
        return

    print(f"\nConnecting to {target.name} at {target.address}...")
    async with BleakClient(target, timeout=30.0) as client:
        print("Connected:", client.is_connected)

        print("\nServices:")
        for service in client.services:
            print("Service:", service.uuid)
            for char in service.characteristics:
                print("  Characteristic:", char.uuid, char.properties)

        print(f"\nStarting notifications on {CHAR_UUID}")
        await client.start_notify(CHAR_UUID, notification_handler)

        print("Receiving data. Press Ctrl+C to stop.\n")

        prev_time = time.perf_counter()
        prev_bytes = 0
        prev_packets = 0
        prev_lost = 0

        try:
            while True:
                await asyncio.sleep(1.0)

                now = time.perf_counter()
                elapsed = now - prev_time

                bytes_diff = total_bytes - prev_bytes
                pkts_diff = total_packets - prev_packets
                lost_diff = lost_packets - prev_lost

                bitrate_bps = (bytes_diff * 8) / elapsed
                bitrate_mbps = bitrate_bps / 1_000_000.0

                avg_pkt_size = (bytes_diff / pkts_diff) if pkts_diff else 0

                print(
                    f"Rate: {bitrate_mbps:.3f} Mbps | "
                    f"{bytes_diff} B/s | "
                    f"{pkts_diff} pkts/s | "
                    f"avg {avg_pkt_size:.1f} B/pkt | "
                    f"lost {lost_diff}"
                )

                prev_time = now
                prev_bytes = total_bytes
                prev_packets = total_packets
                prev_lost = lost_packets

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            await client.stop_notify(CHAR_UUID)
            print("Notifications stopped.")


if __name__ == "__main__":
    asyncio.run(main())