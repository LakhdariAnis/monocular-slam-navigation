# Pi Layer

The Raspberry Pi 3B (1 GB RAM) handles all hardware I/O for the autonomous car: motors, IMU, and camera. SLAM and navigation run entirely on the PC — the Pi is too resource-constrained to run any computer-vision pipeline in real time. Three lightweight services run concurrently on the Pi and communicate with the PC over WiFi via ZMQ (camera, IMU) and HTTP (motor commands).

---

## Why Pi 3B

The 1 GB RAM ceiling is the binding constraint. ORB-SLAM3 requires roughly 800 MB at runtime — after OS overhead that leaves essentially nothing for navigation logic, Python event loops, or concurrent sensor publishing. Attempting to co-locate SLAM on the Pi would cause OOM kills or thrash the SD card with swap, making real-time pose estimation impossible.

**Decision:** offload all compute to the PC. The Pi is treated as a dumb peripheral bus — it owns the GPIO rail, I²C bus, and CSI camera interface but makes no planning decisions. All ZMQ sockets are `PUB`-only or receive HTTP; the Pi never initiates a connection to the PC.

---

## Services

### `car.py` — Motor Controller

Flask HTTP server listening on **port 5000**. The PC navigator sends drive commands as JSON `POST` requests to `/drive`.

**Request format:**

```json
{
  "w": true,
  "a": false,
  "s": false,
  "d": false,
  "inner": 30
}
```

`w/a/s/d` are booleans mapping to `forward / spin-left / reverse / spin-right`. `inner` is the duty cycle (0–100%) applied to the inside wheels during a curve turn — the default is `30` (% duty cycle), producing a tighter arc. The outer wheels always run at 100%.

GPIO control uses the `RPi.GPIO` library in BCM mode. The differential drive layout uses 4 TT motors across 2 × L298N H-bridge drivers with individual PWM speed control at 1 kHz:

| Motor | Enable (PWM) | IN_fwd | IN_rev |
|-------|-------------|--------|--------|
| Front-Right (FR) | GPIO 12 (ENA) | GPIO 17 (IN1) | GPIO 27 (IN2) |
| Front-Left  (FL) | GPIO 13 (ENB) | GPIO 22 (IN3) | GPIO 23 (IN4) |
| Rear-Left   (RL) | GPIO 18 (ENA2)| GPIO 24 (IN5) | GPIO 25 (IN6) |
| Rear-Right  (RR) | GPIO 19 (ENB2)| GPIO 5  (IN7) | GPIO 6  (IN8) |

On-axis spins (`a` or `d` alone) counter-rotate the left/right pairs with no differential reduction — both sides run at full duty cycle.

---

### `imu_zmq.py` — IMU Publisher

Reads the **MPU-6050** over I²C (bus 1, address `0x68`) at **50 Hz** (`DT = 0.02 s`). On startup it averages 200 samples at rest to compute per-axis bias offsets for both the accelerometer and gyroscope.

**Complementary filter for heading:**

```
heading = α × (heading + gyro_z × dt) + (1 - α) × accel_heading
```

The actual α value in code is **`ALPHA = 0.98`**, weighting the gyroscope heavily for short-term smoothness while bleeding in the accelerometer long-term to arrest drift.

#### IMU drift problem

The gyroscope integrates angular velocity to maintain heading. Integration accumulates sensor noise — even a stationary sensor will show slow heading creep (gyro drift). The complementary filter counteracts this: the gyro term (`α = 0.98`) is accurate over short intervals, and the accelerometer term (`1 − α = 0.02`) is drift-free over long intervals, keeping the estimate anchored. Two test scripts were used to characterize real-world drift:

- **`test_imu_live.py`** — logged raw gyro drift rate while stationary.
- **`test_imu_nav.py`** — ran IMU-only dead-reckoning navigation (no SLAM) to quantify accumulated heading error over a full run.

**Result:** drift is acceptable for spin durations under approximately 5 seconds, which is the longest spin the navigation system ever commands (a 90° correction). Beyond that duration, heading error grows faster than the filter can compensate, and SLAM relocalization becomes necessary.

The script also implements **ZUPT** (Zero-velocity UPdaTe): a sliding-window variance detector on `ax`/`ay` that zeros velocity when the car is stationary, preventing dead-reckoning position from drifting during pauses.

**Publishes JSON on ZMQ PUB port 5556:**

```json
{
  "timestamp_ms": 1718000000000,
  "raw_ax":  0.00312,
  "raw_ay": -0.00145,
  "raw_az":  0.98701,
  "raw_gx":  0.0031,
  "raw_gy": -0.0012,
  "raw_gz":  0.0004,
  "heading_deg": 271.34,
  "moving": false,
  "speed_ms": 0.0,
  "x": 0.2341,
  "y": -0.1123
}
```

`heading_deg` is always in `[0, 360)`. `moving` reflects the ZUPT state. `x`/`y` are dead-reckoning position in metres relative to startup.

---

### `camera_zmq.py` — Camera Publisher

Captures from the **Pi Camera v2** via `picamera2` at **640 × 480** pixels. The loop runs as fast as the camera pipeline allows with no explicit FPS cap — throughput is bounded by I²C encoder latency and the ZMQ send buffer. Frames are JPEG-encoded at quality 80 before transmission.

Publishes raw **JPEG bytes** on **ZMQ PUB port 5555**. No topic prefix — `bridge.py` on the PC subscribes with an empty filter and decodes each message as a JPEG buffer.

The capture format is `BGR888` (not the default `RGB888`). This was a deliberate fix: `picamera2`'s default output is RGB, which causes OpenCV to display color-swapped frames; explicitly requesting BGR matches OpenCV's native byte order, so no conversion is needed before passing frames to ORB-SLAM3.

---

## Network Map

| Service | Transport | Port | Direction | Format |
|---------|-----------|------|-----------|--------|
| Camera | ZMQ PUB | 5555 | Pi → PC | JPEG bytes |
| IMU | ZMQ PUB | 5556 | Pi → PC | JSON |
| SLAM poses | ZMQ PUB | 5557 | PC → navigator | JSON |
| Motor commands | HTTP POST | 5000 | PC → Pi | JSON |

All ZMQ sockets bind on Pi-side (`tcp://*:<port>`). The PC connects as subscriber. Motor commands travel in the opposite direction: the PC's navigator issues HTTP POST to `http://<pi-ip>:5000/drive`.

---

## Setup

```bash
# Install dependencies on Pi
pip install flask pyzmq smbus2 picamera2

# Run all three services (one terminal each)
python3 car.py          # terminal 1 — motor HTTP server on :5000
python3 imu_zmq.py      # terminal 2 — IMU ZMQ publisher on :5556
python3 camera_zmq.py   # terminal 3 — camera ZMQ publisher on :5555
```

`imu_zmq.py` prints a live readout of heading, ZUPT state, and variance on startup — keep the terminal visible to confirm the sensor is publishing before starting the PC-side pipeline.

---

## Hardware Wiring Notes

**MPU-6050 → Pi I²C**
- SDA → GPIO 2 (physical pin 3)
- SCL → GPIO 3 (physical pin 5)
- VCC → 3.3 V, GND → GND
- AD0 low → I²C address `0x68`

**L298N motor drivers → Pi GPIO**
- Front driver: ENA=GPIO 12, IN1=GPIO 17, IN2=GPIO 27, ENB=GPIO 13, IN3=GPIO 22, IN4=GPIO 23
- Rear driver:  ENA=GPIO 18, IN5=GPIO 24, IN6=GPIO 25, ENB=GPIO 19, IN7=GPIO 5,  IN8=GPIO 6
- ENA/ENB are hardware PWM-capable pins; `RPi.GPIO.PWM` drives them at 1 kHz

**Pi Camera v2 → CSI connector**
- Standard 15-pin flat cable into the Pi's CSI-2 port
- Ensure the cable is seated with the blue tab facing the USB ports

> **Note:** `car_node.py` exists in this directory but was never used in production. It is a ROS-style node wrapper written during an earlier architecture iteration. Ignore it.
