import time
from smbus2 import SMBus, i2c_msg
import csv
import numpy as np
import scipy.signal as signal

 
 
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
    duration = 60  # seconds
 
    start_time = time.time()
    with SMBus(BUSNUM) as bus:
        reset(bus)
        init_spo2(bus)
 
        red_numbers = []
        IR_numbers = []
 
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
                red_numbers.append(red)
                IR_numbers.append(ir)
 
            time.sleep(0.02)
 
        # ----------------------------
        # Post-processing
        # ----------------------------
        red_np = np.asarray(red_numbers, dtype=float)
        ir_np = np.asarray(IR_numbers, dtype=float)
        N = len(red_np)
 
        if N < 40:
            print("Not enough samples collected for reliable filtering.")
            return
 
        fs = N / duration  # estimated sampling frequency
        print(f"Collected {N} samples, estimated fs = {fs:.2f} Hz")
 
        t = np.linspace(0, duration, N)
 
        # 1) Band-pass filter around heart-rate band
        lowcut = 0.7   # Hz (~42 bpm)
        highcut = 3.5  # Hz (~210 bpm)
        b, a = signal.butter(4, [lowcut, highcut], btype='bandpass', fs=fs)
 
        filtered_red = signal.filtfilt(b, a, red_np)
        filtered_IR = signal.filtfilt(b, a, ir_np)
 
        # 2) "Instant" AC/DC ratio based on global mean (your older style, but with band-pass)
        DC_filtered_red_global = np.mean(filtered_red)
        DC_filtered_IR_global = np.mean(filtered_IR)
        DC_red_global = np.mean(red_np)
        DC_IR_global = np.mean(ir_np)
 
        filtered_R_instant = (np.abs(filtered_red - DC_filtered_red_global) / DC_filtered_red_global) / \
                             (np.abs(filtered_IR - DC_filtered_IR_global) / DC_filtered_IR_global)
        unfiltered_R_instant = (np.abs(red_np - DC_red_global) / DC_red_global) / \
                               (np.abs(ir_np - DC_IR_global) / DC_IR_global)
 
        # 3) Better AC/DC using sliding windows (more stable R)
        dc_window_s = 4.0   # DC baseline window
        ac_window_s = 4.0   # AC RMS window (can be the same)
 
        dc_red = moving_average(red_np, dc_window_s, fs)
        dc_ir = moving_average(ir_np, dc_window_s, fs)
        ac_red = running_rms(filtered_red, ac_window_s, fs)
        ac_ir = running_rms(filtered_IR, ac_window_s, fs)
 
        eps = 1e-8
        R_windowed = (ac_red / (dc_red + eps)) / (ac_ir / (dc_ir + eps))
 
        # 4) Polynomial calibration -> SpO2
        a_coef = 1.5958422
        b_coef = -34.6596622
        c_coef = 112.6898759
 
        spo2_unfiltered_instant = a_coef * unfiltered_R_instant**2 + \
                                   b_coef * unfiltered_R_instant + c_coef
 
        spo2_filtered_instant = a_coef * filtered_R_instant**2 + \
                                b_coef * filtered_R_instant + c_coef
 
        spo2_windowed = a_coef * R_windowed**2 + \
                        b_coef * R_windowed + c_coef
                        
 
        # NOTE: No clipping here – you get the full raw-ish values.
        print("Average_Spo2 " + str(np.mean(spo2_windowed)))
        
 
        # 5) Save CSV with multiple SpO2 variants
        with open('data15.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'SpO2_unfiltered_instant',
                'SpO2_filtered_instant',
                'SpO2_windowed'
            ])
 
            for s_unf, s_filt_inst, s_win in zip(
                    spo2_unfiltered_instant,
                    spo2_filtered_instant,
                    spo2_windowed):
                writer.writerow([s_unf, s_filt_inst, s_win])
 
        print("CSV file 'data2.csv' has been created with 3 SpO2 variants.")
 
 
 
if __name__ == "__main__":
    main()
