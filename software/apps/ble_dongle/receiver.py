# Install: pip3 install bleak
# Run: python3 ./bleak.py

import asyncio
from bleak import BleakScanner

scanner = BleakScanner()

"""
def detection_callback(*args):
    # Ideally we can read adv data with-in this callback,
    # But I tried and found nothing useful inside "args"
    print(args[0])

scanner.register_detection_callback(detection_callback)
"""

async def run():
    """
    await scanner.start()
    await asyncio.sleep(1)
    await scanner.stop()
    devices = await scanner.get_discovered_devices()
    for d in devices:
       print(d)
    """
    while(1):
        # timeout < 1 seems not very stable on my system
        d = await scanner.find_device_by_address("C0:98:E5:49:53:54", timeout=2)
        if not d:
            # Device not found,
            continue
        if ('manufacturer_data' not in d.metadata) or \
           (d.metadata['manufacturer_data'] is None) or \
           (736 not in d.metadata['manufacturer_data']):
            print("Corrupted data %s ..." % d.metadata)
            continue
        # This is the mfg data (without the leading 2-byte mfg id)
        # No idea what is 736
        print(d.metadata['manufacturer_data'][736])

print("Now sanning....")
loop = asyncio.get_event_loop()
loop.run_until_complete(run())
