import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "ESP32_BLE_TPUT"

async def main():
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

    print(f"\nTrying to connect to {target.address}")
    client = BleakClient(target.address, timeout=30.0)

    try:
        await client.connect()
        print("Connected:", client.is_connected)

        print("\nServices:")
        for service in client.services:
            print("Service:", service.uuid)
            for char in service.characteristics:
                print("  Characteristic:", char.uuid, char.properties)

        await client.disconnect()
        print("\nDisconnected")

    except Exception as e:
        print("Connect failed:", repr(e))

asyncio.run(main())