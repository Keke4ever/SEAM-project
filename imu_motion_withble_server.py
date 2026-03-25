# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries (modified for motion detection + BLE)
# SPDX-License-Identifier: MIT
#
# Runs on Raspberry Pi 3
# Reads BNO055 over I2C, detects motion events, and broadcasts them over BLE
# so the ESP32 client can display them on the round screen.
#
# Install dependencies:
#   pip install adafruit-circuitpython-bno055 bluepy
#   sudo apt install python3-dbus
#   pip install dbus-python bless
#
# BLE message format (sent as UTF-8 notify):
#   "MOVING,<lin_mag>"   e.g. "MOVING,4.23"
#   "STILL"
#   "IMPACT,<lin_mag>"   e.g. "IMPACT,11.50"
#   "FREEFALL"

import time
import math
import asyncio
import threading
import logging

import board
import adafruit_bno055

from bless import BlessServer, BlessGATTCharacteristic, GATTCharacteristicProperties, GATTAttributePermissions

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── BLE UUIDs  (must match ESP32 client) ─────────────────────────────────────
MOTION_SERVICE_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
MOTION_CHAR_UUID    = "a1b2c3d4-e5f6-7890-abcd-ef1234567891"

# ── I2C / Sensor Setup ────────────────────────────────────────────────────────
i2c    = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

# ── Tunable Thresholds ────────────────────────────────────────────────────────
MOTION_ACCEL_THRESHOLD = 1.5    # m/s²
IMPACT_THRESHOLD       = 8.0    # m/s²
FREEFALL_THRESHOLD     = 0.8    # m/s²
ORIENTATION_THRESHOLD  = 5.0    # degrees
GYRO_THRESHOLD         = 0.3    # rad/s
LINEAR_DOMINANT_RATIO  = 2.5
STILL_DURATION         = 2.0    # s
SAMPLE_RATE            = 0.1    # s  (10 Hz)

# ── ANSI colours ──────────────────────────────────────────────────────────────
RESET  = "\033[0m";  BOLD  = "\033[1m";  CYAN   = "\033[96m"
YELLOW = "\033[93m"; RED   = "\033[91m"; GREEN  = "\033[92m"; DIM = "\033[2m"

def banner(colour, label, msg):
    print(f"{BOLD}{colour}{label}{RESET}  {msg}")

# ── Shared BLE state ──────────────────────────────────────────────────────────
ble_server    = None
ble_char      = None
ble_ready     = threading.Event()   # set once GATT server is advertising

def send_ble(message: str):
    """Send a UTF-8 notify to connected clients (non-blocking)."""
    if ble_char is None:
        return
    try:
        ble_char.value = message.encode("utf-8")
        if ble_server:
            ble_server.update_value(MOTION_SERVICE_UUID, MOTION_CHAR_UUID)
        logger.info(f"BLE → {message}")
    except Exception as e:
        logger.warning(f"BLE send failed: {e}")

# ── BLE Server (runs in its own thread via asyncio) ───────────────────────────
async def run_ble_server():
    global ble_server, ble_char

    server = BlessServer(name="PiMotion")
    server.read_request_func  = lambda c, **kw: c.value
    server.write_request_func = lambda c, v, **kw: None

    await server.add_new_service(MOTION_SERVICE_UUID)

    char_flags = (GATTCharacteristicProperties.read |
                  GATTCharacteristicProperties.notify)
    permissions = GATTAttributePermissions.readable

    await server.add_new_characteristic(
        MOTION_SERVICE_UUID,
        MOTION_CHAR_UUID,
        char_flags,
        b"STILL",
        permissions
    )

    ble_char   = server.get_characteristic(MOTION_CHAR_UUID)
    ble_server = server

    await server.start()
    logger.info("BLE server advertising as 'PiMotion'")
    ble_ready.set()

    # Keep the event loop alive
    while True:
        await asyncio.sleep(1)

def ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_ble_server())

# ── Helpers ───────────────────────────────────────────────────────────────────
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

def accel_bar(lin_mag, max_val=20.0, width=20):
    filled = int(min(lin_mag / max_val, 1.0) * width)
    return f"[{'█'*filled}{'░'*(width-filled)}] {lin_mag:.2f} m/s²"

def calibration_status():
    s, g, a, m = sensor.calibration_status
    return (f"SYS:{s} GYRO:{g} ACCEL:{a} MAG:{m}  "
            f"({'CALIBRATED' if s == 3 else 'calibrating…'})")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"{BOLD}BNO055 Motion Detection + BLE Server — starting up{RESET}")
    print(f"{DIM}Press Ctrl+C to stop.{RESET}\n")

    # Start BLE in background thread
    t = threading.Thread(target=ble_thread, daemon=True)
    t.start()

    print("Waiting for BLE server to start…")
    ble_ready.wait(timeout=10)
    print(f"{GREEN}BLE ready — advertising as 'PiMotion'{RESET}\n")

    prev_euler       = (0.0, 0.0, 0.0)
    last_motion_time = time.monotonic()
    was_moving       = False
    was_freefall     = False

    try:
        while True:
            now = time.monotonic()

            linear_accel = sensor.linear_acceleration
            raw_accel    = sensor.acceleration
            euler        = sensor.euler
            gyro         = sensor.gyro

            if None in (linear_accel, raw_accel, euler, gyro):
                print("Waiting for sensor data…")
                time.sleep(SAMPLE_RATE)
                continue

            lin_mag   = magnitude(linear_accel)
            raw_mag   = magnitude(raw_accel)
            gyro_mag  = magnitude(gyro)
            ori_delta = euler_delta(prev_euler, euler)

            gyro_equiv = gyro_mag * 0.5
            is_linear  = (lin_mag > MOTION_ACCEL_THRESHOLD and
                          lin_mag > gyro_equiv * LINEAR_DOMINANT_RATIO)

            # 1. Free-fall
            if raw_mag < FREEFALL_THRESHOLD:
                if not was_freefall:
                    banner(RED, "[FREE-FALL]", f"total accel = {raw_mag:.2f} m/s²")
                    send_ble("FREEFALL")
                    was_freefall = True
            else:
                was_freefall = False

            # 2. Impact
            if lin_mag > IMPACT_THRESHOLD:
                banner(RED, "[IMPACT]  ", f"linear accel = {lin_mag:.2f} m/s²")
                send_ble(f"IMPACT,{lin_mag:.2f}")

            # 3. Motion start / stop
            in_motion = lin_mag > MOTION_ACCEL_THRESHOLD or ori_delta > ORIENTATION_THRESHOLD

            if in_motion:
                last_motion_time = now
                if not was_moving:
                    banner(GREEN, "[MOTION START]", f"lin={lin_mag:.2f} m/s²  Δori={ori_delta:.1f}°")
                    send_ble(f"MOVING,{lin_mag:.2f}")
                    was_moving = True

            elif was_moving and (now - last_motion_time) > STILL_DURATION:
                banner(DIM, "[MOTION STOP]", f"still for {STILL_DURATION}s")
                send_ble("STILL")
                was_moving = False

            # 4. Rotation (terminal only — not sent over BLE)
            if gyro_mag > GYRO_THRESHOLD and not is_linear:
                print(f"{DIM}[ROTATION]  gyro = {gyro_mag:.3f} rad/s{RESET}")

            # 5. Linear accel display
            if is_linear:
                x, y, z = [(v or 0.0) for v in linear_accel]
                print(f"{BOLD}{CYAN}[LINEAR ACCEL] {accel_bar(lin_mag)}{RESET}")
                print(f"{CYAN}               X={x:+.2f}  Y={y:+.2f}  Z={z:+.2f} m/s²{RESET}")
            elif in_motion:
                h, r, p = [(v or 0.0) for v in euler]
                print(f"{DIM}  → heading={h:.1f}°  roll={r:.1f}°  pitch={p:.1f}°"
                      f"  | gyro={gyro_mag:.3f} rad/s{RESET}")

            # Calibration reminder
            sys_cal = sensor.calibration_status[0]
            if sys_cal < 3 and int(now) % 5 == 0:
                print(f"{YELLOW}  [CAL] {calibration_status()}{RESET}")

            prev_euler = euler
            time.sleep(SAMPLE_RATE)

    except KeyboardInterrupt:
        print(f"\n{BOLD}{YELLOW}Stopped by user (Ctrl+C). Goodbye!{RESET}\n")

if __name__ == "__main__":
    main()
