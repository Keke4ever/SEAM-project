# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries (modified for motion detection)
# SPDX-License-Identifier: MIT

import time
import math
import board
import adafruit_bno055

# ── I2C / Sensor Setup ────────────────────────────────────────────────────────
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

# ── Tunable Thresholds ────────────────────────────────────────────────────────
MOTION_ACCEL_THRESHOLD   = 1.5    # m/s² — linear accel magnitude to flag motion
IMPACT_THRESHOLD         = 8.0    # m/s² — hard impact / tap detection
FREEFALL_THRESHOLD       = 1.0    # m/s² — total accel below this = free-fall
ORIENTATION_THRESHOLD    = 5.0    # degrees — Euler angle change to flag rotation
GYRO_THRESHOLD           = 0.3    # rad/s — gyro magnitude to flag spinning
STILL_DURATION           = 2.0    # seconds of no motion before "still" reported
SAMPLE_RATE              = 0.1    # seconds between samples (10 Hz)

# ── State ─────────────────────────────────────────────────────────────────────
prev_euler          = (0.0, 0.0, 0.0)
last_motion_time    = time.monotonic()
was_moving          = False
was_freefall        = False


# ── Helpers ───────────────────────────────────────────────────────────────────
def magnitude(vec):
    """Return the Euclidean magnitude of a 3-tuple, treating None values as 0."""
    return math.sqrt(sum((v or 0.0) ** 2 for v in vec))


def euler_delta(e1, e2):
    """
    Return the maximum absolute change across heading, roll, and pitch.
    Handles the 0°/360° wrap-around on heading.
    """
    deltas = []
    for i, (a, b) in enumerate(zip(e1, e2)):
        a = a or 0.0
        b = b or 0.0
        diff = abs(a - b)
        if i == 0:                      # heading wraps at 360°
            diff = min(diff, 360.0 - diff)
        deltas.append(diff)
    return max(deltas)


def calibration_status():
    """Return a human-readable calibration string."""
    sys, gyro, accel, mag = sensor.calibration_status
    return (f"SYS:{sys} GYRO:{gyro} ACCEL:{accel} MAG:{mag}  "
            f"({'CALIBRATED' if sys == 3 else 'calibrating…'})")


# ── Main Loop ─────────────────────────────────────────────────────────────────
print("BNO055 Motion Detection — starting up")
print("Move the sensor to calibrate (figure-8 for magnetometer).\n")

while True:
    now = time.monotonic()

    # ── Raw sensor reads ──────────────────────────────────────────────────────
    linear_accel  = sensor.linear_acceleration  # gravity-removed
    raw_accel     = sensor.acceleration          # includes gravity
    euler         = sensor.euler                 # (heading, roll, pitch)
    gyro          = sensor.gyro

    # Guard against None (sensor still booting)
    if None in (linear_accel, raw_accel, euler, gyro):
        print("Waiting for sensor data…")
        time.sleep(SAMPLE_RATE)
        continue

    # ── Derived magnitudes ────────────────────────────────────────────────────
    lin_mag   = magnitude(linear_accel)   # motion without gravity
    raw_mag   = magnitude(raw_accel)      # total acceleration
    gyro_mag  = magnitude(gyro)
    ori_delta = euler_delta(prev_euler, euler)

    # ── Detection logic ───────────────────────────────────────────────────────

    # 1. Free-fall  (total accel near zero → nothing pulling/pushing sensor)
    if raw_mag < FREEFALL_THRESHOLD:
        if not was_freefall:
            print(f"[FREE-FALL]   total accel = {raw_mag:.2f} m/s²")
            was_freefall = True
    else:
        was_freefall = False

    # 2. Impact / tap  (sudden large linear acceleration spike)
    if lin_mag > IMPACT_THRESHOLD:
        print(f"[IMPACT]      linear accel = {lin_mag:.2f} m/s²  ← hard hit / tap")

    # 3. General motion  (moderate linear acceleration or orientation change)
    in_motion = lin_mag > MOTION_ACCEL_THRESHOLD or ori_delta > ORIENTATION_THRESHOLD

    if in_motion:
        last_motion_time = now
        if not was_moving:
            print(f"[MOTION START]  lin={lin_mag:.2f} m/s²  Δori={ori_delta:.1f}°")
            was_moving = True

    elif was_moving and (now - last_motion_time) > STILL_DURATION:
        print(f"[MOTION STOP]   device still for {STILL_DURATION}s")
        was_moving = False

    # 4. Rotation / spin  (gyroscope magnitude)
    if gyro_mag > GYRO_THRESHOLD:
        print(f"[ROTATION]    gyro = {gyro_mag:.3f} rad/s")

    # 5. Orientation snapshot  (only while moving, every sample)
    if in_motion:
        h, r, p = [(v or 0.0) for v in euler]
        print(f"  → heading={h:.1f}°  roll={r:.1f}°  pitch={p:.1f}°  "
              f"| lin={lin_mag:.2f} m/s²  gyro={gyro_mag:.3f} rad/s")

    # ── Calibration reminder (printed every ~5 s while uncalibrated) ──────────
    sys_cal = sensor.calibration_status[0]
    if sys_cal < 3 and int(now) % 5 == 0:
        print(f"  [CAL] {calibration_status()}")

    # ── Update state ──────────────────────────────────────────────────────────
    prev_euler = euler
    time.sleep(SAMPLE_RATE)
