from bluepy.btle import Scanner, Peripheral

SVC_UUID = "6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10"  # The service UUID

def discover_esp32():
    scanner = Scanner()
    print("Scanning for nearby BLE devices...")
    devices = scanner.scan(10)  # Scans for 10 seconds

    for dev in devices:
        print(f"Found device {dev.addr}, Name: {dev.getValueText(9)}")

        # Check if the service UUID is advertised by the device
        if SVC_UUID in dev.getValueText(8):
            print(f"Found ESP32 with address: {dev.addr}")
            return dev.addr  # Return the MAC address of the ESP32
    
    print("ESP32 not found.")
    return None

def connect_to_esp32(mac_address):
    try:
        print(f"Connecting to ESP32 with address: {mac_address}")
        device = Peripheral(mac_address)  # Connect to ESP32
        print(f"Connected to ESP32: {device.addr}")
        return device
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def main():
    esp32_mac = discover_esp32()  # Discover ESP32 MAC address
    if esp32_mac:
        device = connect_to_esp32(esp32_mac)
        if device:
            # You can now interact with the ESP32 and send/receive data
            print("Interacting with the ESP32...")
            # Send data or read from characteristics
            device.disconnect()
        else:
            print("Could not connect to ESP32.")
    else:
        print("ESP32 not discovered. Retrying...")

if __name__ == "__main__":
    main()
