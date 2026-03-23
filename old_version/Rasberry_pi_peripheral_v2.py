from bluezero import peripheral
from bluezero import adapter
import struct
import threading
import time


def on_connect(device):
    """Called automatically when a central connects."""
    global is_connected, client_address
    is_connected   = True
    client_address = device
    print(f'\n[CONNECTED] Client connected : {device}')
    print(f'[INFO] Ready to send → Heart rate"\n')


def on_disconnect(adapter, device):
    """Called automatically when a central disconnects."""
    global is_connected, client_address
    is_connected   = False
    client_address = None
    print(f'\n[DISCONNECTED] Client disconnected : {device}')
    print('[INFO] Waiting for new connection...\n')


def read_callback():
    """
   
    """
    encoded = "10,15".encode('utf-8')

    print(f'[READ]  Sending → HR')
    return encoded 


def main():
    # Auto-detect Bluetooth adapter
    adapter_address = list(adapter.Adapter.available())[0].address
    is_connected  = False

    # Create peripheral
    ble = peripheral.Peripheral(
        adapter_addr=adapter_address,
        local_name='PI- Three',
        appearance=0
    )
    ble.on_connect    = on_connect
    ble.on_disconnect = on_disconnect

    # Add service
    ble.add_service(
        srv_id=1,
        uuid='6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10',
        primary=True
    )



        

    ble.add_characteristic(
        srv_id=1,
        chr_id=1,
        uuid='c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11',
        value=[],
        notifying=True,
        flags=['read', 'notify'],
        read_callback=read_callback
    )

    print('[INFO] Waiting for client to connect')
    ble.publish()

    print("BLE Peripheral Running...")
    ble.run()

if __name__ == '__main__':
    main()
