"""
Live_data_to_screen_V8.py
=========================
Raspberry Pi 3 — single BLE peripheral (PI-3)
  • MAX30102  → HR + SpO2  (I2C)
  • BNO055    → Motion detection (I2C)

BLE characteristic format  (one notify every 2 s):
    "HR,SPO2,MOTION_STATE,LIN_MAG"
    e.g.  "72,98,MOVING,3.45"
          "68,97,STILL,0.00"
          "80,96,IMPACT,11.20"
          "75,98,FREEFALL,0.00"

IMU is calibrated ONCE at startup — the calibration offsets are saved and
re-applied on every subsequent run so the figure-8 dance is only needed once.

Install:
    pip install smbus2 bluezero adafruit-circuitpython-bno055
    sudo apt install python3-dbus
"""

import numpy as np
import scipy.signal as signal
import scipy.stats as stats
import time
import math
import os
import json
import threading
import queue
import sys
import signal as sig

from smbus2 import SMBus, i2c_msg
import board
import busio
import adafruit_bno055

from bluezero import async_tools, peripheral, adapter

# ═══════════════════════════════════════════════════════════════════════════════
# GLOBAL STOP EVENT
# ═══════════════════════════════════════════════════════════════════════════════
stop_evt = threading.Event()

# ═══════════════════════════════════════════════════════════════════════════════
# MAX30102 — registers & helpers
# ═══════════════════════════════════════════════════════════════════════════════
BUSNUM            = 1
ADDR              = 0x57
REG_INT_STATUS1   = 0x00
REG_INT_ENABLE1   = 0x02
REG_FIFO_WR_PTR   = 0x04
REG_OVF_COUNTER   = 0x05
REG_FIFO_RD_PTR   = 0x06
REG_FIFO_DATA     = 0x07
REG_FIFO_CONFIG   = 0x08
REG_MODE_CONFIG   = 0x09
REG_SPO2_CONFIG   = 0x0A
REG_LED1_PA       = 0x0C   # RED
REG_LED2_PA       = 0x0D   # IR
REG_LED3_PA       = 0x0E   # GREEN (unused)

def write_reg(bus, reg, val):
    bus.write_i2c_block_data(ADDR, reg, [val & 0xFF])

def read_regs(bus, reg, n):
    bus.i2c_rdwr(i2c_msg.write(ADDR, [reg]))
    r = i2c_msg.read(ADDR, n)
    bus.i2c_rdwr(r)
    return list(r)

def reset_max(bus):
    write_reg(bus, REG_MODE_CONFIG, 0x40)
    time.sleep(0.1)
    _ = read_regs(bus, REG_INT_STATUS1, 1)

def init_spo2(bus):
    write_reg(bus, REG_FIFO_WR_PTR, 0x00)
    write_reg(bus, REG_OVF_COUNTER, 0x00)
    write_reg(bus, REG_FIFO_RD_PTR, 0x00)
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
    r  = ((data[0] << 16) | (data[1] << 8) | data[2]) & 0x3FFFF
    ir = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0x3FFFF
    return r, ir

# ═══════════════════════════════════════════════════════════════════════════════
# DSP helpers
# ═══════════════════════════════════════════════════════════════════════════════
SAMPLE_HZ  = 50
WINDOW_SEC = 5
WINDOW_N   = int(SAMPLE_HZ * WINDOW_SEC)

def moving_average(x, window_seconds, fs):
    w = max(1, int(window_seconds * fs))
    return np.convolve(x, np.ones(w) / w, mode='same')

def running_rms(x, window_seconds, fs):
    w = max(1, int(window_seconds * fs))
    return np.sqrt(np.convolve(x * x, np.ones(w) / w, mode='same'))

def calc_spo2(raw_red, raw_ir, filt_red, filt_ir):
    dc_red = moving_average(np.array(raw_red),  1, SAMPLE_HZ)
    dc_ir  = moving_average(np.array(raw_ir),   1, SAMPLE_HZ)
    ac_red = running_rms(np.array(filt_red),     1, SAMPLE_HZ)
    ac_ir  = running_rms(np.array(filt_ir),      1, SAMPLE_HZ)
    eps    = 1e-8
    R      = (ac_red / (dc_red + eps)) / (ac_ir / (dc_ir + eps))
    spo2   = 1.5958422 * R**2 - 34.6596622 * R + 112.6898759
    valid  = spo2[spo2 < 100]
    result = stats.trim_mean(valid, proportiontocut=0.1) if len(valid) else 0.0
    print(f"[SpO2] {result:.1f}%")
    return result

def calc_hr(filt_red):
    peaks, _ = signal.find_peaks(filt_red, distance=20)
    hr = (60 / WINDOW_SEC) * len(peaks)
    print(f"[HR]   {hr:.0f} bpm")
    return hr

# ═══════════════════════════════════════════════════════════════════════════════
# BNO055 — calibration + motion detection
# ═══════════════════════════════════════════════════════════════════════════════
CAL_FILE = os.path.expanduser("~/.bno055_cal.json")

MOTION_ACCEL_THRESHOLD = 1.5    # m/s²
IMPACT_THRESHOLD       = 8.0    # m/s²
FREEFALL_THRESHOLD     = 1.0    # m/s²
ORIENTATION_THRESHOLD  = 5.0    # degrees
LINEAR_DOMINANT_RATIO  = 2.5
STILL_DURATION         = 2.0    # s
IMU_SAMPLE_RATE        = 0.1    # s (10 Hz)

def magnitude(vec):
    return math.sqrt(sum((v or 0.0) ** 2 for v in vec))

def euler_delta(e1, e2):
    deltas = []
    for i, (a, b) in enumerate(zip(e1, e2)):
        a = a or 0.0; b = b or 0.0
        diff = abs(a - b)
        if i == 0:
            diff = min(diff, 360.0 - diff)
        deltas.append(diff)
    return max(deltas)

def save_calibration(imu):
    """Save BNO055 calibration offsets to disk."""
    try:
        offsets = imu.offsets_accelerometer, imu.offsets_gyroscope, imu.offsets_magnetometer
        data = {
            "accel": list(offsets[0]),
            "gyro":  list(offsets[1]),
            "mag":   list(offsets[2]),
        }
        with open(CAL_FILE, "w") as f:
            json.dump(data, f)
        print(f"[IMU] Calibration saved → {CAL_FILE}")
    except Exception as e:
        print(f"[IMU] Could not save calibration: {e}")

def load_calibration(imu):
    """Load and apply saved BNO055 calibration offsets. Returns True if loaded."""
    if not os.path.exists(CAL_FILE):
        return False
    try:
        with open(CAL_FILE) as f:
            data = json.load(f)
        imu.offsets_accelerometer = tuple(data["accel"])
        imu.offsets_gyroscope     = tuple(data["gyro"])
        imu.offsets_magnetometer  = tuple(data["mag"])
        print(f"[IMU] Calibration loaded from {CAL_FILE} — skipping calibration routine")
        return True
    except Exception as e:
        print(f"[IMU] Could not load calibration ({e}) — will calibrate fresh")
        return False

def wait_for_calibration(imu):
    """
    Block until the BNO055 reports full system calibration (sys==3).
    Skipped entirely if saved offsets were loaded successfully.
    """
    print("[IMU] Starting calibration — move sensor in figure-8 pattern…")
    print("      (this only happens once; offsets will be saved for next run)")
    while not stop_evt.is_set():
        sys_c, gyro_c, accel_c, mag_c = imu.calibration_status
        print(f"[IMU CAL] SYS:{sys_c} GYRO:{gyro_c} ACCEL:{accel_c} MAG:{mag_c}", end="\r")
        if sys_c == 3:
            print("\n[IMU] Fully calibrated!")
            save_calibration(imu)
            return
        time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════════════════
# SHARED STATE
# ═══════════════════════════════════════════════════════════════════════════════
raw_q      = queue.Queue(maxsize=4000)
data_queue = queue.Queue()   # items: "HR,SPO2,MOTION,LIN_MAG"

# Motion state (written by IMU thread, read by BLE thread)
_mot_lock      = threading.Lock()
_motion_state  = "STILL"
_motion_linmag = 0.0

def set_motion(state: str, linmag: float = 0.0):
    global _motion_state, _motion_linmag
    with _mot_lock:
        _motion_state  = state
        _motion_linmag = linmag

def get_motion():
    with _mot_lock:
        return _motion_state, _motion_linmag

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD 1 — MAX30102 sensor acquisition
# ═══════════════════════════════════════════════════════════════════════════════
def sensor_thread():
    period = 1.0 / 100
    with SMBus(BUSNUM) as bus:
        reset_max(bus)
        init_spo2(bus)
        next_tick = time.time()
        print(f"[MAX30102] Reading at 0x{ADDR:02X}…")
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
                next_tick = time.time()
    print("[MAX30102] Thread stopped")

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD 2 — HR/SpO2 processing
# ═══════════════════════════════════════════════════════════════════════════════
def processing_thread():
    lowcut = 0.7; highcut = 3.5
    b, a = signal.butter(4, [lowcut, highcut], btype='bandpass', fs=SAMPLE_HZ)
    ir_buf = []; red_buf = []

    while not stop_evt.is_set():
        try:
            t_ms, ir, red = raw_q.get(timeout=0.25)
        except queue.Empty:
            continue
        ir_buf.append(ir); red_buf.append(red)
        if len(ir_buf) < WINDOW_N:
            continue

        filt_ir  = signal.filtfilt(b, a, ir_buf)
        filt_red = signal.filtfilt(b, a, red_buf)
        hr   = calc_hr(filt_red)
        spo2 = calc_spo2(red_buf, ir_buf, filt_red, filt_ir)

        mot_state, lin_mag = get_motion()
        payload = f"{int(hr)},{int(spo2)},{mot_state},{lin_mag:.2f}"
        data_queue.put(payload)
        print(f"[DATA] {payload}")

        # Slide window by 50 samples (1 s)
        if len(ir_buf) >= WINDOW_N:
            del ir_buf[:50]; del red_buf[:50]

    print("[Processing] Thread stopped")

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD 3 — BNO055 motion detection
# ═══════════════════════════════════════════════════════════════════════════════
def imu_thread():
    # BNO055 uses the same I2C bus; adafruit library manages its own handle
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    imu     = adafruit_bno055.BNO055_I2C(i2c_bus)

    # ── One-time calibration ──────────────────────────────────────────────────
    if not load_calibration(imu):
        wait_for_calibration(imu)
    if stop_evt.is_set():
        return

    # ── Motion detection loop ─────────────────────────────────────────────────
    prev_euler       = (0.0, 0.0, 0.0)
    last_motion_time = time.monotonic()
    was_moving       = False
    was_freefall     = False

    print("[IMU] Motion detection running…")
    while not stop_evt.is_set():
        now = time.monotonic()

        lin_acc  = imu.linear_acceleration
        raw_acc  = imu.acceleration
        euler    = imu.euler
        gyro     = imu.gyro

        if None in (lin_acc, raw_acc, euler, gyro):
            time.sleep(IMU_SAMPLE_RATE)
            continue

        lin_mag   = magnitude(lin_acc)
        raw_mag   = magnitude(raw_acc)
        gyro_mag  = magnitude(gyro)
        ori_delta = euler_delta(prev_euler, euler)

        gyro_equiv = gyro_mag * 0.5
        is_linear  = (lin_mag > MOTION_ACCEL_THRESHOLD and
                      lin_mag > gyro_equiv * LINEAR_DOMINANT_RATIO)

        # Free-fall
        if raw_mag < FREEFALL_THRESHOLD:
            if not was_freefall:
                print(f"[IMU] FREE-FALL  raw={raw_mag:.2f} m/s²")
                set_motion("FREEFALL", 0.0)
                was_freefall = True
        else:
            was_freefall = False

        # Impact
        if lin_mag > IMPACT_THRESHOLD:
            print(f"[IMU] IMPACT  lin={lin_mag:.2f} m/s²")
            set_motion("IMPACT", lin_mag)

        # General motion
        in_motion = (lin_mag > MOTION_ACCEL_THRESHOLD or
                     ori_delta > ORIENTATION_THRESHOLD)

        if in_motion:
            last_motion_time = now
            if not was_moving:
                print(f"[IMU] MOTION START  lin={lin_mag:.2f} m/s²  Δori={ori_delta:.1f}°")
                was_moving = True
            if is_linear:
                set_motion("MOVING", lin_mag)
            else:
                set_motion("MOVING", lin_mag)
        elif was_moving and (now - last_motion_time) > STILL_DURATION:
            print("[IMU] STILL")
            set_motion("STILL", 0.0)
            was_moving = False

        prev_euler = euler
        time.sleep(IMU_SAMPLE_RATE)

    print("[IMU] Thread stopped")

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD 4 — BLE peripheral  (single service, single characteristic)
# ═══════════════════════════════════════════════════════════════════════════════
is_connected   = False
client_address = None

def on_connect(device):
    global is_connected, client_address
    is_connected   = True
    client_address = device
    print(f"\n[BLE] Client connected: {device}")

def on_disconnect(adp, device):
    global is_connected, client_address
    is_connected   = False
    client_address = None
    print(f"\n[BLE] Client disconnected: {device}")

def get_latest_payload():
    """Return the most recent combined payload as a UTF-8 byte list."""
    try:
        if data_queue.empty():
            # No HR/SpO2 yet — still send current motion state
            mot_state, lin_mag = get_motion()
            message = f"0,0,{mot_state},{lin_mag:.2f}"
        else:
            latest = None
            while not data_queue.empty():
                latest = data_queue.get_nowait()
            message = latest
        return list(message.encode("utf-8"))
    except Exception as e:
        print(f"[BLE] get_latest_payload error: {e}")
        return list("0,0,STILL,0.00".encode("utf-8"))

def read_callback():
    print("[BLE] Read request")
    return get_latest_payload()

def update_value(characteristic):
    characteristic.set_value(get_latest_payload())
    return characteristic.is_notifying

def notify_callback(notifying, characteristic):
    if notifying:
        async_tools.add_timer_seconds(2, update_value, characteristic)
        print("[BLE] Notify started")

def peripheral_ble():
    time.sleep(1)   # let sensor threads warm up

    adapter_address = list(adapter.Adapter.available())[0].address
    print(f"[BLE] Adapter: {adapter_address}")

    ble = peripheral.Peripheral(
        adapter_address=adapter_address,
        local_name="PI-3",
        appearance=0,
    )

    ble.on_connect    = on_connect
    ble.on_disconnect = on_disconnect

    # Single service — same UUIDs the ESP32 already knows
    ble.add_service(
        srv_id=1,
        uuid="6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10",
        primary=True,
    )

    ble.add_characteristic(
        srv_id=1,
        chr_id=1,
        uuid="c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11",
        value=[],
        notifying=False,
        flags=["read", "notify"],
        read_callback=read_callback,
        notify_callback=notify_callback,
    )

    print("[BLE] Advertising as PI-3 — waiting for client…")
    ble.publish()

# ═══════════════════════════════════════════════════════════════════════════════
# SIGNAL HANDLER
# ═══════════════════════════════════════════════════════════════════════════════
def handle_sig(signum, frame):
    print("\n[Main] Signal received — stopping…")
    stop_evt.set()

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    sig.signal(sig.SIGINT,  handle_sig)
    sig.signal(sig.SIGTERM, handle_sig)

    t1 = threading.Thread(target=sensor_thread,     daemon=True, name="MAX30102")
    t2 = threading.Thread(target=processing_thread, daemon=True, name="DSP")
    t3 = threading.Thread(target=imu_thread,        daemon=True, name="BNO055")
    t4 = threading.Thread(target=peripheral_ble,    daemon=True, name="BLE")

    t1.start()
    time.sleep(0.5)   # let MAX30102 init before DSP thread starts
    t2.start()
    t3.start()        # IMU calibration blocks here until done (or loads from file)
    t4.start()

    print("[Main] All threads running. Ctrl+C to stop.")
    try:
        while not stop_evt.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_evt.set()

    print("[Main] Stopping…")
    time.sleep(0.5)
    sys.exit(0)

if __name__ == "__main__":
    main()
