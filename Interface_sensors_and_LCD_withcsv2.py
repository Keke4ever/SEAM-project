"""
Interface_sensors_and_LCD_V4.py
=================================
Raspberry Pi 3 — single BLE peripheral (PI-3)
  • MAX30102  → HR + SpO2  (I2C)
  • BNO055    → Motion detection (I2C)

BLE characteristic format  (one notify every 2 s):
    "HR,SPO2,ACTIVITY,FALL_STATE"
    e.g.  "72,98,WALKING,NORMAL"
          "68,97,STILL,NORMAL"
          "80,96,RUNNING,NORMAL"
          "75,98,JUMPING,NORMAL"
          "65,96,MOVING,SLIP_FALL"
          "70,97,MOVING,FALL"

ACTIVITY values : STILL | WALKING | RUNNING | JUMPING | MOVING
FALL_STATE values: NORMAL | FALL | SLIP_FALL

Activity classification uses a sliding window of acceleration peaks
(step cadence) plus magnitude bands:
  - WALKING  : ~1–3 steps/s, moderate vertical accel (1.5–5 m/s²)
  - RUNNING  : ~2.5–4 steps/s, higher vertical accel (5–15 m/s²)
  - JUMPING  : brief freefall + large impact spike > 12 m/s²
  - FALL     : freefall (raw_mag < 1 m/s²) followed by large impact
  - SLIP_FALL: sudden large lateral accel + rapid orientation change

IMU calibration is saved once and reloaded on every subsequent run.

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
import collections

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
# BNO055 — calibration
# ═══════════════════════════════════════════════════════════════════════════════
CAL_FILE = os.path.expanduser("~/.bno055_cal.json")

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
# ACTIVITY CLASSIFIER
# ═══════════════════════════════════════════════════════════════════════════════
# Sensor is worn on the wrist/torso. The BNO055 runs at ~10 Hz (IMU_SAMPLE_RATE=0.1s).
# We keep a 3-second rolling window of linear acceleration magnitudes to count
# "steps" (periodic peaks) and measure average/peak magnitude.
#
# Thresholds (tunable):
FREEFALL_RAW_THRESH   = 1.0    # m/s²  — raw accel magnitude for freefall
IMPACT_THRESH         = 6.0   # m/s²  — lin accel spike = hard landing / fall
JUMP_AIRTIME_MIN      = 0.03   # s     — minimum freefall gap to count as jump
JUMP_AIRTIME_MAX      = 0.80   # s     — longer than this → likely a fall
SLIP_LATERAL_THRESH   = 3.0    # m/s²  — sudden side accel for slip detection
SLIP_ORI_THRESH       = 30.0   # °     — rapid orientation change (roll/pitch)
WALK_CADENCE_MIN      = 1.0    # steps/s
WALK_CADENCE_MAX      = 2.8    # steps/s
RUN_CADENCE_MIN       = 2.5    # steps/s
RUN_CADENCE_MAX       = 5.0    # steps/s
WALK_ACCEL_MAX        = 5.0    # m/s²  — average lin mag while walking
RUN_ACCEL_MIN         = 4.0    # m/s²  — average lin mag while running
STILL_LIN_THRESH      = 0.8    # m/s²  — below this = still
STILL_DURATION        = 2.0    # s     — seconds still before STILL state
IMU_SAMPLE_RATE       = 0.1    # s  (10 Hz)
IMU_HZ                = int(1.0 / IMU_SAMPLE_RATE)
WINDOW_SAMPLES        = IMU_HZ * 3  # 3-second window = 30 samples

class ActivityClassifier:
    """
    Stateful classifier that runs every IMU sample.
    Call update() each cycle; read .activity and .fall_state.
    """
    def __init__(self):
        self.lin_mag_buf   = collections.deque(maxlen=WINDOW_SAMPLES)
        self.raw_mag_buf   = collections.deque(maxlen=WINDOW_SAMPLES)
        self.euler_buf     = collections.deque(maxlen=WINDOW_SAMPLES)

        # Fall / jump tracking state machine
        self._freefall_start  = None   # monotonic time when freefall began
        self._in_freefall     = False
        self._last_impact_mag = 0.0
        self._fall_cooldown   = 0.0    # time until fall alert expires

        # Still timer
        self._last_motion_time = time.monotonic()

        # Outputs
        self.activity   = "STILL"
        self.fall_state = "NORMAL"

    # ── helpers ────────────────────────────────────────────────────────────────
    def _count_steps(self):
        """Count periodic peaks in linear accel magnitude over the buffer."""
        if len(self.lin_mag_buf) < IMU_HZ:
            return 0.0
        arr = np.array(self.lin_mag_buf)
        # Minimum peak height and spacing to separate walking/running steps
        peaks, _ = signal.find_peaks(arr, height=STILL_LIN_THRESH, distance=max(2, IMU_HZ // 5))
        duration_s = len(arr) / IMU_HZ
        return len(peaks) / duration_s if duration_s > 0 else 0.0

    def _avg_lin_mag(self):
        if not self.lin_mag_buf:
            return 0.0
        return float(np.mean(list(self.lin_mag_buf)))

    def _peak_lin_mag(self):
        if not self.lin_mag_buf:
            return 0.0
        return float(np.max(list(self.lin_mag_buf)))

    def _lateral_component(self, lin_acc):
        """Magnitude of horizontal (X,Y) acceleration (ignores vertical Z)."""
        x = lin_acc[0] or 0.0
        y = lin_acc[1] or 0.0
        return math.sqrt(x*x + y*y)

    # ── main update ────────────────────────────────────────────────────────────
    def update(self, lin_acc, raw_acc, euler):
        now = time.monotonic()

        lin_mag  = magnitude(lin_acc)
        raw_mag  = magnitude(raw_acc)
        lat_mag  = self._lateral_component(lin_acc)

        self.lin_mag_buf.append(lin_mag)
        self.raw_mag_buf.append(raw_mag)
        self.euler_buf.append(euler)

        # ── Orientation change rate ─────────────────────────────────────────
        if len(self.euler_buf) >= 3:
            ori_change = euler_delta(self.euler_buf[-3], self.euler_buf[-1])
        else:
            ori_change = 0.0

        # ── Fall state machine ──────────────────────────────────────────────
        # Priority: active fall cooldown → freefall+impact detection → slip detection

        # Expire fall alert after 3 s
        if self.fall_state != "NORMAL" and now > self._fall_cooldown:
            self.fall_state = "NORMAL"
            print("[ACT] Fall alert expired → NORMAL")

        # Detect freefall start
        if raw_mag < FREEFALL_RAW_THRESH and not self._in_freefall:
            self._in_freefall    = True
            self._freefall_start = now
            print(f"[ACT] Freefall start  raw_mag={raw_mag:.2f}")

        elif self._in_freefall:
            airtime = now - self._freefall_start

            if raw_mag >= FREEFALL_RAW_THRESH:
                # Freefall ended — classify by airtime and subsequent impact
                self._in_freefall = False

                if lin_mag > IMPACT_THRESH:
                    if airtime < JUMP_AIRTIME_MIN:
                        # Too short to be a real jump — treat as heavy footstep / normal
                        pass
                    elif airtime <= JUMP_AIRTIME_MAX:
                        # Classic jump: brief airtime + hard landing
                        print(f"[ACT] JUMP detected  airtime={airtime:.2f}s  impact={lin_mag:.1f}")
                        self.activity   = "JUMPING"
                        # Not a fall — deliberate jump
                    else:
                        # Long airtime + hard impact = fall
                        print(f"[ACT] FALL detected  airtime={airtime:.2f}s  impact={lin_mag:.1f}")
                        self.fall_state    = "FALL"
                        self._fall_cooldown = now + 3.0
                        self.activity      = "MOVING"
                else:
                    # Short freefall, low impact — could just be a stumble or step gap
                    if airtime > JUMP_AIRTIME_MAX:
                        print(f"[ACT] FALL (soft impact) airtime={airtime:.2f}s")
                        self.fall_state    = "FALL"
                        self._fall_cooldown = now + 3.0
                        self.activity      = "MOVING"

        # Slip detection: sudden lateral accel + rapid orientation change
        if (self.fall_state == "NORMAL" and
                lat_mag > SLIP_LATERAL_THRESH and
                ori_change > SLIP_ORI_THRESH):
            print(f"[ACT] SLIP_FALL  lat={lat_mag:.1f}  ori_Δ={ori_change:.1f}°")
            self.fall_state    = "SLIP_FALL"
            self._fall_cooldown = now + 3.0
            self.activity      = "MOVING"

        # ── Activity classification (only when not in fall alert) ───────────
        if self.fall_state == "NORMAL":
            if lin_mag > STILL_LIN_THRESH:
                self._last_motion_time = now

            if (now - self._last_motion_time) > STILL_DURATION:
                self.activity = "STILL"
            else:
                cadence  = self._count_steps()
                avg_lin  = self._avg_lin_mag()
                peak_lin = self._peak_lin_mag()

                if self.activity == "JUMPING":
                    # Stay in JUMPING briefly (will decay to other states next cycle)
                    pass
                elif cadence >= RUN_CADENCE_MIN and avg_lin >= RUN_ACCEL_MIN:
                    self.activity = "RUNNING"
                elif WALK_CADENCE_MIN <= cadence <= WALK_CADENCE_MAX and avg_lin <= WALK_ACCEL_MAX:
                    self.activity = "WALKING"
                elif lin_mag > STILL_LIN_THRESH:
                    self.activity = "MOVING"
                else:
                    self.activity = "STILL"

        return self.activity, self.fall_state

# ═══════════════════════════════════════════════════════════════════════════════
# SHARED STATE
# ═══════════════════════════════════════════════════════════════════════════════
raw_q      = queue.Queue(maxsize=4000)
data_queue = queue.Queue()   # items: "HR,SPO2,ACTIVITY,FALL_STATE"

_mot_lock    = threading.Lock()
_activity    = "STILL"
_fall_state  = "NORMAL"

def set_motion(activity: str, fall_state: str):
    global _activity, _fall_state
    with _mot_lock:
        _activity   = activity
        _fall_state = fall_state

def get_motion():
    with _mot_lock:
        return _activity, _fall_state

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
        header = ['IR', 'RED']

        start_time = time.monotonic()
        while not stop_evt.is_set():
            n = available_samples(bus)
            if n == 0:
                time.sleep(0.01)
                continue
            now_ms = int(time.monotonic() - start_time) & 0xFFFFFFFF
            for _ in range(n):
                red, ir = read_sample(bus)
                time.sleep(1)
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
        with open('Raw_data_3.csv', 'a', newline='') as f:
            writer = csv.writer(f)
           
            header= ["Time", "IR", "Red"]
            if f.tell() == 0:
                writer.writerow(header)

            try:
                t_ms, ir, red = raw_q.get(timeout=0.25)
                data_point = [t_ms ,ir, red]
                writer.writerow(data_point)
                f.flush() # Forces writing to disk immediately
                os.fsync(f.fileno())
            except queue.Empty:
                continue
        ir_buf.append(ir); red_buf.append(red)
        if len(ir_buf) < WINDOW_N:
            continue

        filt_ir  = signal.filtfilt(b, a, ir_buf)
        filt_red = signal.filtfilt(b, a, red_buf)
        hr   = calc_hr(filt_red)
        spo2 = calc_spo2(red_buf, ir_buf, filt_red, filt_ir)

        activity, fall_state = get_motion()
        payload = f"{int(hr)},{int(spo2)},{activity},{fall_state}"
        data_queue.put(payload)
        print(f"[DATA] {payload}")

        if len(ir_buf) >= WINDOW_N:
            del ir_buf[:50]; del red_buf[:50]

    print("[Processing] Thread stopped")

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD 3 — BNO055 motion + activity detection
# ═══════════════════════════════════════════════════════════════════════════════
def imu_thread():
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    imu     = adafruit_bno055.BNO055_I2C(i2c_bus)

    if not load_calibration(imu):
        wait_for_calibration(imu)
    if stop_evt.is_set():
        return

    classifier = ActivityClassifier()
    prev_activity  = ""
    prev_fall      = ""

    print("[IMU] Activity detection running…")
    while not stop_evt.is_set():
        lin_acc  = imu.linear_acceleration
        raw_acc  = imu.acceleration
        euler    = imu.euler

        if None in (lin_acc, raw_acc, euler):
            time.sleep(IMU_SAMPLE_RATE)
            continue

        activity, fall_state = classifier.update(lin_acc, raw_acc, euler)
        set_motion(activity, fall_state)

        if activity != prev_activity or fall_state != prev_fall:
            print(f"[IMU] {activity} | {fall_state}")
            prev_activity = activity
            prev_fall     = fall_state

        time.sleep(IMU_SAMPLE_RATE)

    print("[IMU] Thread stopped")

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD 4 — BLE peripheral
# ═══════════════════════════════════════════════════════════════════════════════
is_connected   = False
client_address = None
_notify_active = False

def on_connect(device):
    global is_connected, client_address
    is_connected   = True
    client_address = device
    print(f"\n[BLE] Client connected: {device}")

def on_disconnect(adp, device):
    global is_connected, client_address, _notify_active
    is_connected   = False
    client_address = None
    _notify_active = False
    print(f"\n[BLE] Client disconnected: {device}")

def get_latest_payload():
    try:
        latest = None
        while True:
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            message = latest
        else:
            activity, fall_state = get_motion()
            message = f"0,0,{activity},{fall_state}"

        return list(message.encode("utf-8"))
    except Exception as e:
        print(f"[BLE] get_latest_payload error: {e}")
        return list("0,0,STILL,NORMAL".encode("utf-8"))

def read_callback():
    print("[BLE] Read request")
    return get_latest_payload()

def update_value(characteristic):
    if not is_connected:
        return False
    try:
        characteristic.set_value(get_latest_payload())
    except Exception as e:
        print(f"[BLE] update_value error: {e}")
    return True

def notify_callback(notifying, characteristic):
    global _notify_active
    if notifying and not _notify_active:
        _notify_active = True
        async_tools.add_timer_seconds(2, update_value, characteristic)
        print("[BLE] Notify started")
    elif not notifying:
        _notify_active = False
        print("[BLE] Notify stopped")

def peripheral_ble():
    time.sleep(1)

    adapter_address = list(adapter.Adapter.available())[0].address
    print(f"[BLE] Adapter: {adapter_address}")

    ble = peripheral.Peripheral(
        adapter_address=adapter_address,
        local_name="PI-3",
        appearance=0,
    )

    ble.on_connect    = on_connect
    ble.on_disconnect = on_disconnect

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
    time.sleep(0.5)
    t2.start()
    t3.start()
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