import pandas
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
import scipy.stats as stats
import time
from smbus2 import SMBus, i2c_msg
import csv


 
 
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

def main():
    duration = 5 # seconds
 
    start_time = time.time()
    with SMBus(BUSNUM) as bus:
        reset(bus)
        init_spo2(bus)
 
        Raw_red = []
        Raw_IR = []
 
        print(f"Reading MAX30101 at 0x{ADDR:02X}... (Ctrl+C to stop)")
 
        # ----------------------------
        # Acquisition loop
        # ----------------------------
        while time.time() - start_time < duration:
            n = available_samples(bus)
            if n == 0:
                time.sleep(0.01)
                continue
 
            for _ in range(n):
                red, ir = read_sample(bus)
                Raw_red.append(red)
                Raw_IR.append(ir)
 
            time.sleep(0.02)

      
        lowcut = 0.7   # Hz (~42 bpm)
        highcut = 3.5  # Hz (~210 bpm)
        b, a = signal.butter(4, [lowcut, highcut], btype='bandpass', fs=25)
        filtered_red = signal.filtfilt(b, a, Raw_red)
        filtered_IR=signal.filtfilt(b, a, Raw_IR)
        
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
        print("Average_Spo2 " + str(stats.trim_mean(filtered_spo2, proportiontocut=0.1)))

        #heart rate
        peaks_index, peak_heights=signal.find_peaks(filtered_red,  distance=10)
        print("Heart rate:" + str(12*len(peaks_index)))

 
  

 
    

if __name__ == "__main__":
    main()
