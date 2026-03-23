#!/usr/bin/env python3
import time
import struct
import math
import threading
import queue
import signal
import sys

# ---------- CONFIG ----------
I2C_BUS = 1
MAX30101_ADDR = 0x57

# BLE: must match your ESP32 service + characteristic UUIDs
SVC_UUID = "6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10"
CHR_UUID_STREAM = "c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11"

ESP32_NAME_HINT = "XIAO-Round-Display"   # optional name hint; can be None

SAMPLE_HZ = 100              # common for MAX3010x (50-200 ok)
WINDOW_SEC = 8               # processing window for HR/SpO2
WINDOW_N = int(SAMPLE_HZ * WINDOW_SEC)

RAW_Q_MAX = 4000
PROC_Q_MAX = 200

# SpO2 calibration (rough; tune with real data)
# Typical linear map: SpO2 = 110 - 25 * R, where R = (ACred/DCred) / (ACir/DCir)
SPO2_A = 110.0
SPO2_B = 25.0

# ---------- DEPENDENCIES ----------
# pip install smbus2 bleak
from smbus2 import SMBus
from bleak import BleakClient, BleakScanner


# ---------- UTIL: simple IIR filters ----------
class DCTracker:
    """Single-pole low-pass to estimate DC component."""
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.y = None

    def step(self, x: float) -> float:
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha * self.y + (1.0 - self.alpha) * x
        return self.y


class IIRBandpassLite:
    """
    Lightweight “bandpass-like” by subtracting slow DC and smoothing the result.
    This isn't a perfect DSP bandpass, but it's stable and works well for pulse.
    """
    def __init__(self, dc_alpha: float, ac_smooth: float):
        self.dc = DCTracker(dc_alpha)
        self.ac_smooth = ac_smooth
        self.ac_y = 0.0

    def step(self, x: float) -> float:
        dc = self.dc.step(x)
        ac = x - dc
        # smooth AC a bit to reduce noise
        self.ac_y = self.ac_smooth * self.ac_y + (1.0 - self.ac_smooth) * ac
        return self.ac_y


# ---------- MAX30101 minimal driver (enough to read FIFO) ----------
class MAX30101:
    # Registers (subset)
    REG_INTR_STATUS_1 = 0x00
    REG_INTR_STATUS_2 = 0x01
    REG_INTR_ENABLE_1 = 0x02
    REG_INTR_ENABLE_2 = 0x03
    REG_FIFO_WR_PTR   = 0x04
    REG_OVF_COUNTER   = 0x05
    REG_FIFO_RD_PTR   = 0x06
    REG_FIFO_DATA     = 0x07
    REG_FIFO_CONFIG   = 0x08
    REG_MODE_CONFIG   = 0x09
    REG_SPO2_CONFIG   = 0x0A
    REG_LED1_PA       = 0x0C  # IR
    REG_LED2_PA       = 0x0D  # RED
    REG_MULTI_LED_CTRL1 = 0x11
    REG_MULTI_LED_CTRL2 = 0x12

    MODE_SPO2 = 0x03

    def __init__(self, bus: SMBus, addr=MAX30101_ADDR):
        self.bus = bus
        self.addr = addr

    def wreg(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def rreg(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def rblock(self, reg, n):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def reset(self):
        # Reset bit = 1 << 6
        self.wreg(self.REG_MODE_CONFIG, 0x40)
        time.sleep(0.05)

    def init_spo2(self, sample_hz=SAMPLE_HZ):
        """
        Configure for RED+IR via FIFO.
        This is a pragmatic config. You can tune LED currents, sample rate, etc.
        """
        self.reset()

        # Interrupts off for polling
        self.wreg(self.REG_INTR_ENABLE_1, 0x00)
        self.wreg(self.REG_INTR_ENABLE_2, 0x00)

        # FIFO: sample avg=1, rollover enabled, almost full = 17
        # FIFO_CONFIG: [7:5] sample avg, [4] rollover, [3:0] almost full
        self.wreg(self.REG_FIFO_CONFIG, (0 << 5) | (1 << 4) | 0x0F)

        # Mode: SpO2
        self.wreg(self.REG_MODE_CONFIG, self.MODE_SPO2)

        # SPO2_CONFIG:
        # ADC range + sample rate + pulse width
        # sample rate codes: 50/100/200/400/800/1000/1600/3200
        # We'll map requested sample_hz to closest common code.
        sr_code = 0x03  # 100 Hz default
        if sample_hz <= 50: sr_code = 0x02
        elif sample_hz <= 100: sr_code = 0x03
        elif sample_hz <= 200: sr_code = 0x04
        elif sample_hz <= 400: sr_code = 0x05
        elif sample_hz <= 800: sr_code = 0x06
        else: sr_code = 0x07

        adc_range = 0x01  # 8192 nA (reasonable)
        pw_code = 0x03    # 411 us (18-bit) good SNR
        self.wreg(self.REG_SPO2_CONFIG, (adc_range << 5) | (sr_code << 2) | pw_code)

        # LED currents (tune!)
        self.wreg(self.REG_LED1_PA, 0x24)  # IR
        self.wreg(self.REG_LED2_PA, 0x24)  # RED

        # Multi-LED slots (RED + IR):
        # Slot1 = LED1 (IR), Slot2 = LED2 (RED)
        self.wreg(self.REG_MULTI_LED_CTRL1, (0x01 << 4) | 0x02)
        self.wreg(self.REG_MULTI_LED_CTRL2, 0x00)

        # Clear pointers
        self.wreg(self.REG_FIFO_WR_PTR, 0)
        self.wreg(self.REG_OVF_COUNTER, 0)
        self.wreg(self.REG_FIFO_RD_PTR, 0)

    def fifo_samples_available(self) -> int:
        wr = self.rreg(self.REG_FIFO_WR_PTR)
        rd = self.rreg(self.REG_FIFO_RD_PTR)
        return (wr - rd) & 0x1F

    def read_fifo_samples(self, max_samples=32):
        """
        Returns list of (ir, red) raw 18-bit values.
        Each sample in SPO2 mode is 6 bytes: 3 bytes per LED.
        With multi-led slots, order is Slot1 then Slot2.
        """
        n = self.fifo_samples_available()
        if n <= 0:
            return []
        n = min(n, max_samples)

        out = []
        for _ in range(n):
            data = self.rblock(self.REG_FIFO_DATA, 6)
            # 18-bit packed in 3 bytes; top 6 bits unused
            ir  = ((data[0] & 0x03) << 16) | (data[1] << 8) | data[2]
            red = ((data[3] & 0x03) << 16) | (data[4] << 8) | data[5]
            out.append((ir, red))
        return out


# ---------- Processing: HR peak + SpO2 ratio-of-ratios ----------
def estimate_hr_from_ir(ir_ac_buf, fs):
    """
    Simple robust-ish HR estimate:
    - compute adaptive threshold from AC RMS
    - detect peaks with refractory period
    """
    if len(ir_ac_buf) < fs * 3:
        return None

    # RMS as noise/signal proxy
    rms = math.sqrt(sum(x*x for x in ir_ac_buf) / len(ir_ac_buf))
    thresh = max(rms * 0.8, 50.0)  # tune; depends on your scaling

    peaks = []
    refractory = int(0.25 * fs)  # 250ms min between beats (240 bpm max)
    last_peak = -10**9

    for i in range(2, len(ir_ac_buf)-2):
        x = ir_ac_buf[i]
        if x > thresh and x > ir_ac_buf[i-1] and x > ir_ac_buf[i+1]:
            if i - last_peak >= refractory:
                peaks.append(i)
                last_peak = i

    if len(peaks) < 2:
        return None

    # Use median of intervals
    intervals = [peaks[i] - peaks[i-1] for i in range(1, len(peaks))]
    intervals.sort()
    med = intervals[len(intervals)//2]
    if med <= 0:
        return None
    bpm = 60.0 * fs / med
    # sanity clamp
    if bpm < 35 or bpm > 220:
        return None
    return bpm


def estimate_spo2(ir_ac_buf, red_ac_buf, ir_dc_buf, red_dc_buf):
    """
    Ratio-of-ratios:
    R = (ACred/DCred) / (ACir/DCir)
    AC estimated by RMS over window.
    DC estimated by mean of DC trackers.
    """
    if not ir_ac_buf or not red_ac_buf or not ir_dc_buf or not red_dc_buf:
        return None

    ac_ir = math.sqrt(sum(x*x for x in ir_ac_buf) / len(ir_ac_buf))
    ac_red = math.sqrt(sum(x*x for x in red_ac_buf) / len(red_ac_buf))
    dc_ir = sum(ir_dc_buf) / len(ir_dc_buf)
    dc_red = sum(red_dc_buf) / len(red_dc_buf)

    # avoid divide by zero
    if dc_ir <= 1 or dc_red <= 1 or ac_ir <= 1:
        return None

    R = (ac_red / dc_red) / (ac_ir / dc_ir)
    spo2 = SPO2_A - SPO2_B * R
    # clamp realistic range
    spo2 = max(70.0, min(100.0, spo2))
    return spo2


# ---------- Threads ----------
stop_evt = threading.Event()

raw_q = queue.Queue(maxsize=RAW_Q_MAX)     # (t_ms, ir, red)
proc_q = queue.Queue(maxsize=PROC_Q_MAX)   # (t_ms, hr, spo2, ir_dc, red_dc)


def sensor_thread():
    with SMBus(I2C_BUS) as bus:
        dev = MAX30101(bus)
        dev.init_spo2(SAMPLE_HZ)

        next_tick = time.time()
        period = 1.0 / SAMPLE_HZ

        while not stop_evt.is_set():
            # read whatever is available
            samples = dev.read_fifo_samples(max_samples=32)
            now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

            for (ir, red) in samples:
                try:
                    raw_q.put_nowait((now_ms, ir, red))
                except queue.Full:
                    # drop oldest by draining a bit
                    try:
                        raw_q.get_nowait()
                        raw_q.put_nowait((now_ms, ir, red))
                    except queue.Empty:
                        pass

            # pace lightly (FIFO polling doesn’t need exact sleep)
            next_tick += period
            dt = next_tick - time.time()
            if dt > 0:
                time.sleep(min(dt, 0.01))
            else:
                # we're behind; resync
                next_tick = time.time()


def processing_thread():
    # Filters
    # DC alpha: closer to 1.0 = slower DC (good). At 100 Hz, 0.995 ~ ~2s time const-ish
    ir_f = IIRBandpassLite(dc_alpha=0.995, ac_smooth=0.85)
    red_f = IIRBandpassLite(dc_alpha=0.995, ac_smooth=0.85)

    # Rolling buffers
    ir_ac_buf = []
    red_ac_buf = []
    ir_dc_buf = []
    red_dc_buf = []

    last_emit = 0

    while not stop_evt.is_set():
        try:
            t_ms, ir, red = raw_q.get(timeout=0.25)
        except queue.Empty:
            continue

        ir_ac = ir_f.step(float(ir))
        red_ac = red_f.step(float(red))

        # DC trackers are inside filter objects
        ir_dc = ir_f.dc.y if ir_f.dc.y is not None else float(ir)
        red_dc = red_f.dc.y if red_f.dc.y is not None else float(red)

        ir_ac_buf.append(ir_ac)
        red_ac_buf.append(red_ac)
        ir_dc_buf.append(ir_dc)
        red_dc_buf.append(red_dc)

        if len(ir_ac_buf) > WINDOW_N:
            ir_ac_buf.pop(0); red_ac_buf.pop(0); ir_dc_buf.pop(0); red_dc_buf.pop(0)

        # emit at ~10 Hz
        if (t_ms - last_emit) < 100:
            continue
        last_emit = t_ms

        hr = estimate_hr_from_ir(ir_ac_buf, SAMPLE_HZ)
        spo2 = estimate_spo2(ir_ac_buf, red_ac_buf, ir_dc_buf, red_dc_buf)

        # If not enough data yet, still push placeholders so UI can show "warming up"
        hr_out = hr if hr is not None else 0.0
        spo2_out = spo2 if spo2 is not None else 0.0

        # downscale DC to fit in uint16 for payload (very rough)
        ir_dc_u16 = int(max(0, min(65535, ir_dc / 16.0)))
        red_dc_u16 = int(max(0, min(65535, red_dc / 16.0)))

        try:
            proc_q.put_nowait((t_ms, hr_out, spo2_out, ir_dc_u16, red_dc_u16))
        except queue.Full:
            try:
                proc_q.get_nowait()
                proc_q.put_nowait((t_ms, hr_out, spo2_out, ir_dc_u16, red_dc_u16))
            except queue.Empty:
                pass


async def ble_worker():
    """
    BLE central:
    - scan for ESP32 (by name hint if provided)
    - connect
    - write packed payload to characteristic continuously
    """
    while not stop_evt.is_set():
        try:
            device = None

            # Scan
            for _ in range(3):
                if stop_evt.is_set():
                    return
                devices = await BleakScanner.discover(timeout=3.0)
                for d in devices:
                    if ESP32_NAME_HINT and d.name and ESP32_NAME_HINT in d.name:
                        device = d
                        break
                if device:
                    break

            if not device:
                # If you don’t want name matching, you can hardcode MAC address here.
                await asyncio_sleep(1.0)
                continue

            # Connect + stream
            async with BleakClient(device) as client:
                # optional: verify service exists
                svcs = await client.get_services()
                if SVC_UUID.lower() not in [s.uuid.lower() for s in svcs]:
                    # still try to write; some stacks hide until connected
                    pass

                while not stop_evt.is_set() and client.is_connected:
                    try:
                        t_ms, hr, spo2, ir_dc_u16, red_dc_u16 = proc_q.get(timeout=0.5)
                    except queue.Empty:
                        continue

                    hr_x10 = int(max(0, min(65535, round(hr * 10.0))))
                    spo2_x10 = int(max(0, min(65535, round(spo2 * 10.0))))

                    payload = struct.pack(
                        "<IHHHH",
                        t_ms & 0xFFFFFFFF,
                        hr_x10,
                        spo2_x10,
                        int(ir_dc_u16),
                        int(red_dc_u16),
                    )

                    # Write WITHOUT response for speed (ESP32 characteristic must allow it)
                    await client.write_gatt_char(CHR_UUID_STREAM, payload, response=False)

        except Exception:
            # backoff + retry
            await asyncio_sleep(1.0)


# Tiny async sleep helper without importing asyncio everywhere
import asyncio
async def asyncio_sleep(s: float):
    await asyncio.sleep(s)


def ble_thread():
    asyncio.run(ble_worker())


def handle_sig(sig, frame):
    stop_evt.set()


def main():
    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    t1 = threading.Thread(target=sensor_thread, daemon=True)
    t2 = threading.Thread(target=processing_thread, daemon=True)
    t3 = threading.Thread(target=ble_thread, daemon=True)

    t1.start(); t2.start(); t3.start()

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
