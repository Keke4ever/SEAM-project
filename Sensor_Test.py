#This code just prints the raw sensor values

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
        while True:
            n = available_samples(bus)
            if n == 0:
                time.sleep(0.01)
                continue
 
            for _ in range(n):
                red, ir = read_sample(bus)
                print("Raw Red value is: " + str(red) + "Raw IR value is: " + str(ir) )
                red_numbers.append(red)
                IR_numbers.append(ir)
 
            time.sleep(0.02)
 
 
 
 
if __name__ == "__main__":
    main()
