#!/usr/bin/env python3
# imu_odom.py — IMU odometry for Raspberry Pi + MPU6050
# Publishes JSON over ZMQ PUB port 5556 at 50Hz

import smbus2, struct, time, math, collections, json
import zmq

# ── Hardware constants ──────────────────────────────────────────────────────
I2C_ADDR    = 0x68
ACCEL_SCALE = 20575.0
GYRO_SCALE  = 131.0
GRAVITY     = 9.80665

# ── Tunable parameters ──────────────────────────────────────────────────────
DT            = 0.02
CALIB_SAMPLES = 200
WIN           = 10            # reduced from 15 — flushes stale samples faster
VAR_THRESH_ON  = 0.00025     # ZUPT engages  (still  → moving threshold)
VAR_THRESH_OFF = 0.00060     # ZUPT releases (moving → still  threshold)
ALPHA         = 0.98
AVERAGE_SPEED = 0.2367
ZMQ_PORT      = 5556

# ── I2C + ZMQ setup ─────────────────────────────────────────────────────────
bus = smbus2.SMBus(1)
bus.write_byte_data(I2C_ADDR, 0x6B, 0x00)
time.sleep(0.1)

ctx  = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind(f"tcp://*:{ZMQ_PORT}")

# ── Startup calibration ──────────────────────────────────────────────────────
def read_raw():
    raw = bus.read_i2c_block_data(I2C_ADDR, 0x3B, 14)
    return struct.unpack('>7h', bytes(raw))   # ax ay az temp gx gy gz

print(f"Hold still — calibrating {CALIB_SAMPLES} samples...")
ax0 = ay0 = az0 = gx0 = gy0 = gz0 = 0.0
for _ in range(CALIB_SAMPLES):
    v = read_raw()
    ax0 += v[0]; ay0 += v[1]; az0 += v[2]
    gx0 += v[4]; gy0 += v[5]; gz0 += v[6]
    time.sleep(DT)

ax0 /= CALIB_SAMPLES; ay0 /= CALIB_SAMPLES; az0 /= CALIB_SAMPLES
gx0 /= CALIB_SAMPLES; gy0 /= CALIB_SAMPLES; gz0 /= CALIB_SAMPLES
az0 -= ACCEL_SCALE   # remove gravity from Z bias

print(f"Bias: ax={ax0:.1f} ay={ay0:.1f} az={az0:.1f} | "
      f"gx={gx0:.1f} gy={gy0:.1f} gz={gz0:.1f}")
print(f"\nPublishing on ZMQ port {ZMQ_PORT} — Ctrl+C to stop\n")

# ── State ────────────────────────────────────────────────────────────────────
heading_deg = 0.0
x = y       = 0.0
zupt        = True   # start frozen until window fills
win_ax = collections.deque(maxlen=WIN)
win_ay = collections.deque(maxlen=WIN)

t_prev = time.monotonic()

print(f"{'ZUPT':>5} {'hdg°':>7} {'spd':>6} {'x(m)':>8} {'y(m)':>8} {'var':>10}")
print("─" * 52)

# ── Main loop ────────────────────────────────────────────────────────────────
while True:
    t_now  = time.monotonic()
    dt     = t_now - t_prev
    t_prev = t_now

    # 1. Read sensor
    v  = read_raw()
    ax = (v[0] - ax0) / ACCEL_SCALE
    ay = (v[1] - ay0) / ACCEL_SCALE
    az = (v[2] - az0) / ACCEL_SCALE
    gx = (v[4] - gx0) / GYRO_SCALE
    gy = (v[5] - gy0) / GYRO_SCALE
    gz = (v[6] - gz0) / GYRO_SCALE

    # 2. Heading — complementary filter
    heading_deg = heading_deg + gz * dt

    # 3. Variance
    win_ax.append(ax)
    win_ay.append(ay)
    if len(win_ax) == WIN:
        mx  = sum(win_ax) / WIN
        my  = sum(win_ay) / WIN
        var = sum((a - mx)**2 for a in win_ax) / WIN \
            + sum((a - my)**2 for a in win_ay) / WIN
    else:
        var = 0.0   # window not full yet — stay frozen

    # 4. Hysteresis ZUPT — fixes slow release
    if zupt:
        if var > VAR_THRESH_OFF:   # needs strong signal to release
            zupt = False
    else:
        if var < VAR_THRESH_ON:    # normal sensitivity to engage
            zupt = True

    # 5. Position update
    if zupt:
        speed = 0.0
    else:
        speed = AVERAGE_SPEED
        hdg_r = math.radians(heading_deg)
        x    += speed * math.cos(hdg_r) * dt
        y    += speed * math.sin(hdg_r) * dt

    # 6. ZMQ publish
    ts_ms = int(time.time() * 1000)
    msg = json.dumps({
        "timestamp_ms": ts_ms,
        "raw_ax":  round(ax, 5),
        "raw_ay":  round(ay, 5),
        "raw_az":  round(az, 5),
        "raw_gx":  round(gx, 4),
        "raw_gy":  round(gy, 4),
        "raw_gz":  round(gz, 4),
        "heading_deg": round(heading_deg % 360, 2),
        "moving":  not zupt,
        "speed_ms": round(speed, 3),
        "x": round(x, 4),
        "y": round(y, 4),
    })
    sock.send_string(msg)

    # 7. Terminal readout — var shown so you can tune thresholds live
    print(
        f"{'YES' if zupt else ' no':>5} "
        f"{heading_deg % 360:>7.1f} "
        f"{speed:>6.2f} "
        f"{x:>+8.3f} {y:>+8.3f} "
        f"{var:>10.6f}",
        end='\r'
    )

    # 8. Pace loop
    elapsed = time.monotonic() - t_now
    if DT - elapsed > 0:
        time.sleep(DT - elapsed)
