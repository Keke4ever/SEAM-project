import time
import bluetooth

# UUID for the service and characteristic
SVC_UUID = "6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10"  # Same as in your ESP32 code
CHR_UUID = "c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11"  # Same as in your ESP32 code

# Values to send (SpO2 and Heart Rate)
heart_rate = 75  # Example heart rate value
spo2 = 98        # Example SpO2 value

def connect_to_esp32():
    print("Searching for ESP32 device...")
    nearby_devices = bluetooth.discover_devices(lookup_uuids=True)
    
    for addr, name, uuid in nearby_devices:
        if uuid == SVC_UUID:
            print(f"Found ESP32 device with address {addr}")
            return addr
    
    print("ESP32 device not found!")
    return None

def send_data_to_esp32(esp32_address, hr, spo2):
    # Create a Bluetooth socket
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    
    try:
        sock.connect((esp32_address, 1))  # Connect to the ESP32 on channel 1 (RFCOMM)
        print("Connected to ESP32")
        
        # Create the data to send (formatted as HR=XX,SPO2=YY)
        data = f"HR={hr},SPO2={spo2}"
        sock.send(data)  # Send the data to ESP32
        print(f"Sent data: {data}")
        
    except bluetooth.btcommon.BluetoothError as e:
        print(f"Bluetooth connection error: {e}")
    finally:
        sock.close()

def main():
    while True:
        esp32_address = connect_to_esp32()
        if esp32_address:
            send_data_to_esp32(esp32_address, heart_rate, spo2)
            time.sleep(5)  # Wait before sending the next data
        else:
            print("Retrying to find ESP32...")
            time.sleep(2)

if __name__ == "__main__":
    main()
