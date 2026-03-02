import numpy as np
import scipy.signal as signal
import scipy.stats as stats
import time
from smbus2 import SMBus, i2c_msg
import csv
from bluezero import async_tools
from bluezero import peripheral
from bluezero import adapter
import threading
import queue
import sys
import signal as sig

stop_evt = threading.Event()
raw_q = queue.Queue(maxsize=4000)     
data_queue = queue.Queue()
is_notifying = False
SAMPLE_HZ = 100    
WINDOW_SEC = 8               # processing window for HR/SpO2
WINDOW_N = int(SAMPLE_HZ * WINDOW_SEC)


BUSNUM = 1
ADDR = 0x57  # MAX30101 / MAX30102 typical I2C address
 
# ---- Registers (from datasheet) ----
REG_INT_STATUS1 = 0x00
REG_INT_ENABLE1 = 0x02
REG_FIFO_WR_PTR = 0x04
REG_OVF_COUNTER = 0x05
REG_FIFO_RD_PTR = 0x06
REG_FIFO_DATA = 0x07
REG_FIFO_CONFIG = 0x08
REG_MODE_CONFIG = 0x09
REG_SPO2_CONFIG = 0x0A
REG_LED1_PA = 0x0C  # RED
REG_LED2_PA = 0x0D  # IR
REG_LED3_PA = 0x0E  # GREEN (unused here)
 
 
def write_reg(bus, reg, val):
    bus.write_i2c_block_data(ADDR, reg, [val & 0xFF])
 
 
def read_regs(bus, reg, n):
    # Set the register pointer then read n bytes
    bus.i2c_rdwr(i2c_msg.write(ADDR, [reg]))
    r = i2c_msg.read(ADDR, n)
    bus.i2c_rdwr(r)
    return list(r)
 
 
def reset(bus):
    # Set RESET bit in MODE_CONFIG (bit6), self-clears
    write_reg(bus, REG_MODE_CONFIG, 0x40)
    time.sleep(0.1)
    # Clear interrupt status
    _ = read_regs(bus, REG_INT_STATUS1, 1)
 
 
def init_spo2(bus):
    # Clear FIFO pointers/counters
    write_reg(bus, REG_FIFO_WR_PTR, 0x00)
    write_reg(bus, REG_OVF_COUNTER, 0x00)
    write_reg(bus, REG_FIFO_RD_PTR, 0x00)
 
    # FIFO_CONFIG: SMP_AVE=4 (010<<5), ROLLOVER_EN=1<<4, A_FULL=0x0F
    write_reg(bus, REG_FIFO_CONFIG, (0b010 << 5) | (1 << 4) | 0x0F)
 
    # MODE_CONFIG: SpO2 mode (MODE[2:0]=0b011)
    write_reg(bus, REG_MODE_CONFIG, 0x03)
 
    # SpO2_CONFIG: 4096nA range, 100 sps, 411us pulse width
    write_reg(bus, REG_SPO2_CONFIG, 0x27)
 
    # LED currents
    write_reg(bus, REG_LED1_PA, 0x24)  # RED
    write_reg(bus, REG_LED2_PA, 0x24)  # IR
    write_reg(bus, REG_LED3_PA, 0x00)  # GREEN off
 
 
def available_samples(bus):
    wr = read_regs(bus, REG_FIFO_WR_PTR, 1)[0] & 0x1F
    rd = read_regs(bus, REG_FIFO_RD_PTR, 1)[0] & 0x1F
    return (wr - rd) & 0x1F
 
 
def read_sample(bus):
    # One sample in SpO2 mode = RED(3B) + IR(3B) = 6 bytes
    data = read_regs(bus, REG_FIFO_DATA, 6)
    r = ((data[0] << 16) | (data[1] << 8) | data[2]) & 0x3FFFF
    ir = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0x3FFFF
    return r, ir

def moving_average(x, window_seconds, fs):
    """Time-based moving average."""
    w = max(1, int(window_seconds * fs))
    kernel = np.ones(w) / w
    return np.convolve(x, kernel, mode='same')
 
 
def running_rms(x, window_seconds, fs):
    """Time-based running RMS."""
    w = max(1, int(window_seconds * fs))
    kernel = np.ones(w) / w
    x2 = x * x
    return np.sqrt(np.convolve(x2, kernel, mode='same'))

# -------  For bluetooth peripherial for touchscreen -------
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


def get_latest_value():
     
    try:
        # Get latest value without blocking
        # If queue is empty send a default message
        if data_queue.empty():
            print("no data yet")
            message = '0,0'
        else:
            # Drain queue and keep only the latest reading
            latest = None
            while not data_queue.empty():
                latest = data_queue.get_nowait()
            message = latest
           

        encoded = list(message.encode('utf-8'))

    except Exception as e:
        print("Error occured")
        return list('Error'.encode('utf-8'))
    
    return list(encoded)
    
def read_callback():
    message=get_latest_value()
    print("Sending data on read request")
    return message

def update_value(characteristic):
    characteristic.set_value(get_latest_value())
    return characteristic.is_notifying
    
def notify_callback(notifying, charcteristic):
    if notifying:
            async_tools.add_timer_seconds(6, update_value, charcteristic)
            print("just notifies")


def sensor_thread():
        period = 1.0 / 100
        with SMBus(BUSNUM) as bus:
            reset(bus)
            init_spo2(bus)
            next_tick = time.time()
           
    
            print(f"Reading MAX30101 at 0x{ADDR:02X}... (Ctrl+C to stop)")
    
            # ----------------------------
            # Acquisition loop
            # ----------------------------
            while not stop_evt.is_set():
                n = available_samples(bus)
                if n == 0:
                    time.sleep(0.01)
                    continue
                
                now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
                for _ in range(n):
                    red, ir = read_sample(bus)
                    try:
                        raw_q.put_nowait((now_ms, ir, red))
                    except queue.Full:
                        # drop oldest by draining a bit
                        try:
                            raw_q.get_nowait()
                            raw_q.put_nowait((now_ms, ir, red))
                        except queue.Empty:
                            pass
                
                next_tick += period
                dt = next_tick - time.time()
                if dt > 0:
                    time.sleep(min(dt, 0.01))
                else:
                    # we're behind; resync
                    next_tick = time.time()   
                

            time.sleep(0.02)

def SPO2(Raw_red, Raw_IR,filtered_red,filtered_IR):
    
    
    np_filtered_IR= np.array(filtered_IR)
    np_filtered_red= np.array(filtered_red)
    np_unfiltered_red=np.array(Raw_red)
    np_unfiltered_IR=np.array(Raw_IR)



    dc_window_s = 2 # DC baseline window
    ac_window_s = 2 # AC RMS window (can be the same)
    fs=25

    dc_red = moving_average(np_unfiltered_red, dc_window_s, fs)
    dc_ir = moving_average(np_unfiltered_IR, dc_window_s, fs)
    ac_red = running_rms(np_filtered_red, ac_window_s, fs)
    ac_ir = running_rms(np_filtered_IR, ac_window_s, fs)

    eps = 1e-8
    R_windowed = (ac_red / (dc_red + eps)) / (ac_ir / (dc_ir + eps))

    a_coef = 1.5958422
    b_coef = -34.6596622
    c_coef = 112.6898759
    
    spo2_windowed = a_coef * R_windowed**2 + \
                        b_coef * R_windowed + c_coef
    
    

    filtered_spo2=spo2_windowed[(spo2_windowed<100)]
    sp02=stats.trim_mean(filtered_spo2, proportiontocut=0.1)
    print("Average_Spo2 " + str(sp02))
    return sp02

def Heart_rate(filtered_red):
    
    #heart rate
    peaks_index, peak_heights=signal.find_peaks(filtered_red,  distance=10)
    hr = str((60/WINDOW_SEC)*len(peaks_index))
    print("Heart rate:" + str(hr))
    return hr
   

def processing_thread():
    
    lowcut = 0.7   # Hz (~42 bpm)
    highcut = 3.5  # Hz (~210 bpm)
    b, a = signal.butter(4, [lowcut, highcut], btype='bandpass', fs=25)
    
    # Rolling buffers
    ir_raw_buf = []
    red_raw_buf = []


    last_emit = 0

    while not stop_evt.is_set():
        try:
            t_ms, ir, red = raw_q.get(timeout=0.25)
        except queue.Empty:
            continue

        ir_raw_buf.append(ir)
        red_raw_buf.append(red)
       

        if len(ir_raw_buf) > WINDOW_N:
            ir_raw_buf.pop(0); red_raw_buf.pop(0); 

        # emit at ~10 Hz
        if (t_ms - last_emit) < 100:
            continue
        last_emit = t_ms

        filtered_red = signal.filtfilt(b, a, ir_raw_buf)
        filtered_IR=signal.filtfilt(b, a, red_raw_buf)
        hr = Heart_rate(filtered_red, SAMPLE_HZ)
        spo2 = SPO2(red_raw_buf, ir_raw_buf,filtered_red,filtered_IR)
        Data=str(hr) + "," + str(spo2)
        data_queue.put(Data)

def peripheral():
    # Give collector a moment to get first reading
    time.sleep(1)
    # Auto-detect Bluetooth adapter
    adapter_address = list(adapter.Adapter.available())[0].address
    is_connected  = False
    print("Mac: ADDRESS")
    print(adapter_address)

    # Create peripheral
    ble = peripheral.Peripheral(
        adapter_address=adapter_address,
        local_name='PI-3',
        appearance=0,
        
    )
   

    # Add service
    ble.add_service(
        srv_id=1,
        uuid='6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10',
        primary=True
    )

    ble.on_connect= on_connect
    ble.on_disconnect = on_disconnect

    ble.add_characteristic(
        srv_id=1,
        chr_id=1,
        uuid='c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11',
        value= [],
        notifying=False,
        flags=['read','notify'],
        read_callback=read_callback,
        notify_callback=notify_callback
    )

    print('[INFO] Waiting for client to connect')
    print(is_connected)
    ble.publish()
    



def handle_sig(sig, frame):
    stop_evt.set()


def main():
    sig.signal(signal.SIGINT, handle_sig)
    sig.signal(signal.SIGTERM, handle_sig)

    t1 = threading.Thread(target=sensor_thread, daemon=True)
    t2 = threading.Thread(target=processing_thread, daemon=True)
    t3=threading.Thread(target=peripheral, daemon=True)

    t1.start(); t2.start(); t3.start()

    # Give collector a moment to get first reading
    time.sleep(1)
    # Auto-detect Bluetooth adapter
    
    print("SEAM Pi0 streamer running. Ctrl+C to stop.")
    try:
        while not stop_evt.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_evt.set()

    print("Stopping...")
    time.sleep(0.5)
    sys.exit(0)
    

if __name__ == "__main__":
    main()
