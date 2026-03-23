import numpy as np
import scipy.signal as signal
import scipy.stats as stats
import time
import threading
import queue
from collections import deque
from smbus2 import SMBus, i2c_msg

BUSNUM = 1
ADDR = 0x57
FS = 25
WINDOW_SECONDS = 5
WINDOW_SIZE = FS * WINDOW_SECONDS

# ---------------- Queue ----------------
sample_queue = queue.Queue(maxsize=1000)

# ---------------- Registers ----------------
REG_INT_STATUS1 = 0x00
REG_FIFO_WR_PTR = 0x04
REG_OVF_COUNTER = 0x05
REG_FIFO_RD_PTR = 0x06
REG_FIFO_DATA = 0x07
REG_FIFO_CONFIG = 0x08
REG_MODE_CONFIG = 0x09
REG_SPO2_CONFIG = 0x0A
REG_LED1_PA = 0x0C
REG_LED2_PA = 0x0D
REG_LED3_PA = 0x0E

# ---------------- I2C ----------------
def write_reg(bus, reg, val):
    bus.write_i2c_block_data(ADDR, reg, [val & 0xFF])

def read_regs(bus, reg, n):
    bus.i2c_rdwr(i2c_msg.write(ADDR, [reg]))
    r = i2c_msg.read(ADDR, n)
    bus.i2c_rdwr(r)
    return list(r)

def reset(bus):
    write_reg(bus, REG_MODE_CONFIG, 0x40)
    time.sleep(0.1)
    _ = read_regs(bus, REG_INT_STATUS1, 1)

def init_spo2(bus):
    write_reg(bus, REG_FIFO_WR_PTR, 0)
    write_reg(bus, REG_OVF_COUNTER, 0)
    write_reg(bus, REG_FIFO_RD_PTR, 0)
    write_reg(bus, REG_FIFO_CONFIG, (0b010 << 5) | (1 << 4) | 0x0F)
    write_reg(bus, REG_MODE_CONFIG, 0x03)
    write_reg(bus, REG_SPO2_CONFIG, 0x27)
    write_reg(bus, REG_LED1_PA, 0x24)
    write_reg(bus, REG_LED2_PA, 0x24)
    write_reg(bus, REG_LED3_PA, 0x00)

def available_samples(bus):
    wr = read_regs(bus, REG_FIFO_WR_PTR, 1)[0] & 0x1F
    rd = read_regs(bus, REG_FIFO_RD_PTR, 1)[0] & 0x1F
    return (wr - rd) & 0x1F

def read_sample(bus):
    data = read_regs(bus, REG_FIFO_DATA, 6)
    r = ((data[0]<<16)|(data[1]<<8)|data[2]) & 0x3FFFF
    ir = ((data[3]<<16)|(data[4]<<8)|data[5]) & 0x3FFFF
    return r, ir

# ---------------- Producer ----------------
def acquire_data():
    with SMBus(BUSNUM) as bus:
        reset(bus)
        init_spo2(bus)
        print("Acquisition started")

        while True:
            n = available_samples(bus)
            if n == 0:
                time.sleep(0.005)
                continue

            for _ in range(n):
                red, ir = read_sample(bus)
                try:
                    sample_queue.put_nowait((red, ir))
                except queue.Full:
                    pass

# ---------------- Consumer ----------------
def process_data():

    # Precompute Butterworth bandpass
    b, a = signal.butter(4, [0.7, 3.5], btype='bandpass', fs=FS)

    # Filter states
    zi_red = signal.lfilter_zi(b, a)
    zi_ir  = signal.lfilter_zi(b, a)

    # Sliding windows
    red_window = deque(maxlen=WINDOW_SIZE)
    ir_window  = deque(maxlen=WINDOW_SIZE)

    filtered_red_window = deque(maxlen=WINDOW_SIZE)

    while True:
        red, ir = sample_queue.get()

        # Streaming filter (one sample at a time)
        red_filt, zi_red = signal.lfilter(b, a, [red], zi=zi_red)
        ir_filt, zi_ir   = signal.lfilter(b, a, [ir], zi=zi_ir)

        red_window.append(red)
        ir_window.append(ir)
        filtered_red_window.append(red_filt[0])

        if len(red_window) < WINDOW_SIZE:
            continue

        # Convert to numpy for stats
        red_np = np.array(red_window)
        ir_np  = np.array(ir_window)
        red_filt_np = np.array(filtered_red_window)

        # DC components
        dc_red = np.mean(red_np)
        dc_ir  = np.mean(ir_np)

        # AC components (RMS of filtered)
        ac_red = np.std(red_filt_np)
        ac_ir  = np.std(signal.lfilter(b, a, ir_np))

        R = (ac_red/dc_red) / (ac_ir/dc_ir + 1e-8)

        # Quadratic calibration
        spo2 = 1.5958422*R**2 - 34.6596622*R + 112.6898759
        spo2 = np.clip(spo2, 80, 100)

        # Peak detection
        peaks, _ = signal.find_peaks(red_filt_np, distance=FS/2)
        bpm = (len(peaks) * 60) / WINDOW_SECONDS

        print(f"HR: {bpm:.1f} bpm | SpO2: {spo2:.1f}%")

# ---------------- Main ----------------
if __name__ == "__main__":
    t1 = threading.Thread(target=acquire_data, daemon=True)
    t2 = threading.Thread(target=process_data, daemon=True)

    t1.start()
    t2.start()

    while True:
        time.sleep(1)
