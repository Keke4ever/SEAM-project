from bluezero import peripheral
from bluezero import adapter
import struct
import threading
import time

# Auto-detect Bluetooth adapter
adapter_address = list(adapter.Adapter.available())[0].address

# Create peripheral
ble = peripheral.Peripheral(
    adapter_addr=adapter_address,
    local_name='PI- Three',
    appearance=0
)

# Add service
ble.add_service(
    srv_id=1,
    uuid='6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10',
    primary=True
)

# Initial value (two integers packed)
initial_value = "10,15".encode('utf-8')

ble.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid='c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11',
    value=initial_value,
    notifying=True,
    flags=['read', 'notify']
)

# Function to update values periodically
def update_values():
    val1 = 0
    val2 = 10

    while True:
        message = f"{val1},{val2}"
        encoded = message.encode('utf-8')

        ble.characteristics[0].set_value(encoded)
        ble.characteristics[0].notify()

        print("Sent:", message)

        
        val1 += 1
        val2 += 2
        time.sleep(2)

# Start update thread
threading.Thread(target=update_values, daemon=True).start()

ble.publish()

print("BLE Peripheral Running...")
ble.run()