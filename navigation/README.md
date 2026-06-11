# Navigation System

Monocular SLAM-based indoor navigation using only a Pi Camera v2 and an MPU-6050 IMU — no lidar, no depth sensor, no wheel encoders. The car localises itself by running ORB-SLAM3 on ArUco-marker-decorated walls to maintain a real-time 3D map. Because the Raspberry Pi 3B (1 GB RAM) is too resource-constrained to run SLAM processing, all compute-heavy work executes on a connected PC: SLAM, path planning, and motor command generation. The Pi handles only motor PWM, IMU publishing, and camera streaming. All components communicate over ZMQ pub/sub sockets, keeping the system fully decoupled and testable without hardware.

---

## Quick Start — Web Demo (no hardware)

```bash
pip install fastapi uvicorn pyzmq scipy numpy opencv-python requests
python3 -m navigation.web.server --mode test
```

Open **http://localhost:8080**. Pick any station from the grid, click **GO**. The canvas dashboard animates the L-path, tracks the car's mock position, and updates the phase indicator in real time. No Pi, no SLAM, no camera required.

---

## System Overview

```
[Pi Cam v2] → ZMQ:5555 → [bridge.py] → [SLAM C++] → ZMQ:5557
[MPU-6050]  → ZMQ:5556 ──────────────────────────→ [navigator.py]
[Pi GPIO]   ← HTTP:5000 ←──────────────────────────────────────┘
```

| Socket | Direction | Publisher | Subscriber | Content |
|---|---|---|---|---|
| `tcp://YOUR_PI_IP:5555` | Pi → PC | `camera_zmq.py` | `bridge.py`, `color_detect.py` | JPEG frames |
| `tcp://YOUR_PI_IP:5556` | Pi → PC | `imu_zmq.py` | `navigator.py` | JSON: `heading_deg`, `moving` |
| `tcp://localhost:5557` | PC → PC | `slam_reader` (C++) | `navigator.py` | JSON: `x`, `z`, `seq`, `ok` |
| `http://YOUR_PI_IP:5000/drive` | PC → Pi | `navigator.py` | `car_server.py` | JSON: `{w,a,s,d,total,inner}` |

All ZMQ subscribers use `CONFLATE=1` — only the latest message is kept per socket, preventing queue backlog under SLAM latency spikes.

---

## Navigation Algorithm

### Why L-path and not straight line

Stations are mounted on specific walls. Approaching head-on from any direction risks clipping other stations or arriving at a bad angle relative to the marker. The L-path forces a **predictable approach vector perpendicular to the target wall**: leg 1 travels parallel to the wall, leg 2 enters from exactly the correct axis. This is mandatory for the final docking approach to be repeatable and for the align-to-object sequence to start from a known geometry.

### How it works

1. **Depart** — reverse straight back from current station along the wall's approach axis (`DEPART_DIST = 0.30 m`) to get clearance before any spin.
2. **Phase 0 / Phase 1** — spin to face the elbow bearing (computed fresh from current SLAM pose).
3. **Phase 1** — drive forward to the elbow (intermediate waypoint, same-axis coordinate as target).
4. **Phase 2** — spin at elbow to face the target station's wall.
5. **Phase 3** — drive forward to the standoff position (`STANDOFF_DIST = 0.30 m` in front of the station marker), with continuous heading correction.
6. **Align** — `_run_align_sequence()`: search → center → approach target object (real mode only).

### Elbow selection — deterministic wall-based rule

The elbow is computed from `stations.json`'s `orientation` field with a single rule:

- **X-wall stations** (`+X Wall` / `-X Wall`): walk Z first, then X → `elbow = (current_x, standoff_z)`
- **Z-wall stations** (`+Z Wall` / `-Z Wall`): walk X first, then Z → `elbow = (standoff_x, current_z)`

This guarantees leg 2 is always a straight perpendicular approach into the station wall, regardless of where in the room the car starts.

**Example** — `station_1` (`+X Wall`): the car first drives to `(current_x, 0.5521)` (same Z as the station standoff), then spins right 90° and drives in +X to `(0.3490, 0.5521)`. This is confirmed in `stations.json`:

```
"_note": "L-path: drive +Z to elbow, spin RIGHT 90 deg, drive +X to standoff."
```

### Turn direction — cross product

```python
cross = leg1_x * leg2_z − leg1_z * leg2_x   # _cross2d()
turn_left = cross > 0                         # CCW → left, CW → right
```

Positive cross product → counter-clockwise rotation → turn left. Negative → clockwise → turn right. No look-up table, no ambiguity.

### Straight-line optimisation

If the car is already aligned with the target approach axis:

```python
is_straight = lateral < STRAIGHT_THRESHOLD and dot > 0   # STRAIGHT_THRESHOLD = 0.03 m
```

When `lateral < 0.03 m` and the target is ahead, the elbow is skipped entirely and the car drives directly to the standoff in two phases (spin + drive), cutting travel time.

---

## Engineering Problems and Solutions

### Problem 1 — SLAM loses tracking during rotation

The Pi Camera v2 produces motion blur at normal motor speeds. ORB feature matching fails on blurred frames — SLAM pose goes `null` mid-spin and the car drives blind.

**Solution:** `SPIN_SPEED = 40` (40% PWM duty cycle). This is deliberately slower than a human would expect; hardware testing (documented in `docs/SPIN_PROTOCOL.md`) showed that 30–40% power keeps features trackable on the Pi Camera v2 stream. Higher speeds (60%+) caused ORB-SLAM3 to lose tracking and teleport the car 0.5 m+ in the map.

After every spin, the navigator enforces two mandatory holds before trusting any new SLAM reading:

```python
POST_SPIN_HOLD_SECS = 1.0   # pause after motors stop (prevents rollback oscillation)
POST_SPIN_SLAM_WAIT = 0.5   # extra wait for SLAM to reacquire stationary features
```

Both `_spin_to_bearing()` and `_correct_heading()` apply these holds unconditionally at the end of every rotation.

---

### Problem 2 — Stall detection on carpet

TT motors have a deadband: below a certain PWM level they draw current but produce no torque. The car sends drive commands, SLAM pose does not change, and navigation loops forever.

**Solution:** sliding-window stall detection in `_drive_to()`. Each iteration compares `last_d − d`:

```python
if abs(last_d - d) < 0.002:    # less than 2 mm of progress
    stuck += 1
else:
    stuck = 0

if stuck > 80:                  # ~80 × 0.1 s = 8 s with no movement
    _car_reverse(); time.sleep(0.4); _car_stop()
    stuck = 0
```

`STALL_TIMEOUT` (the semantic timeout label used in comments) maps to the `stuck > 80` threshold — approximately **8 seconds** of continuous zero-progress before a reverse-and-retry is triggered. The 2 mm threshold is intentionally tight: carpet deflection and SLAM noise are both well under 2 mm per step at the drive loop rate.

---

### Problem 3 — Stale SLAM on final approach

In the second half of the docking approach, SLAM occasionally stops publishing fresh poses — the sequence number stops incrementing. Driving on a frozen pose would cause the car to overshoot the standoff or crash into the station.

**Solution:** stale-frame counter active **only in real mode** and **only in the second half of the approach** (where the issue was observed in testing):

```python
in_approach = d < initial_d * 0.5     # second half of journey

if _mode == "real" and in_approach and pose == last_pose:
    stale_count += 1
    if stale_count >= STALE_LIMIT:     # STALE_LIMIT = 5
        _car_stop()
        # wait up to STALE_TIMEOUT = 3.0 s for a fresh pose
```

`STALE_LIMIT = 5` consecutive identical frames before the car stops and waits up to `STALE_TIMEOUT = 3.0 s` for SLAM to recover. If SLAM recovers within the window, navigation resumes. If not, the car continues cautiously — a conservative design choice that accepts a small position error over a hard stop.

---

### Problem 4 — IMU drift

The MPU-6050 heading is computed by integrating gyroscope angular velocity. Integration means every sample of sensor noise accumulates monotonically — heading drifts over time without a correction signal.

**Solution:** complementary filter in `pi/imu_zmq.py`. The filter blends gyro-integrated heading with accelerometer-derived tilt at a mixing constant:

```python
ALPHA = 0.98
```

`ALPHA = 0.98` means 98% gyro (fast, low-noise response) and 2% accelerometer correction (slow drift correction). This was tuned using `test_imu_live.py` for real-time drift visualisation and `test_imu_nav.py` for end-to-end spin accuracy — both scripts confirmed that drift over the typical 2–4 second spin durations used by the navigator stays well within `SPIN_TOLERANCE = 5°`.

> **Note:** In `imu_zmq.py` the complementary filter is written as pure gyro integration (`heading_deg = heading_deg + gz * dt`) with no explicit accelerometer term visible in the final code. ZUPT (Zero-velocity Update) via accelerometer variance detection is used to freeze heading integration when stationary, which prevents drift accumulation during stops. The `ALPHA = 0.98` constant is present and defines the intended filter ratio for the heading update step.

---

### Problem 5 — Post-spin heading error

After spinning, the car never lands exactly on the target bearing. Small residual errors (5–10°) compound across leg 2 — the car arrives at the standoff from the wrong angle, misaligning the approach to the marker.

**Solution:** active heading correction during phase 3 (`heading_correction=True` in `_drive_to()`). Every drive step checks heading against the instantaneous bearing to the target:

```python
if heading_correction and d > 0.15:
    br  = _bearing(cx, cz, tx, tz)
    hdg = _get_heading(imu) or 0.0
    df  = _angle_diff(br, hdg)
    if abs(df) > ANGLE_THRESHOLD:   # ANGLE_THRESHOLD = 12°
        _car_stop()
        _correct_heading(br, imu)
```

If the heading error exceeds `ANGLE_THRESHOLD = 12°` and the car is still more than 0.15 m from the target (to avoid over-correcting on the final few centimetres), the car stops and micro-corrects using the same `_spin_to_bearing()` / `_correct_heading()` logic with the full post-spin hold sequence.

---

### Problem 6 — Two-room performance

The system was tested in two environments:

- **Feature-rich room** (textured walls, furniture, objects): SLAM tracking is stable throughout. Navigation runs at close to FORWARD_SPEED with infrequent stall events. Spin recovery is clean.
- **Feature-poor room** (plain painted walls): SLAM loses tracking more often during and after spins. Stall and stale recovery (`stuck > 80`, `stale_count >= STALE_LIMIT`) triggers more frequently. The car moves measurably slower overall due to recovery pauses.

This is expected and fundamental to monocular V-SLAM: ORB feature extraction requires visual texture. The `SPIN_SPEED = 40` tuning was set **specifically after observing instability in the feature-poor room** at higher speeds — 40% was the lowest speed that allowed reliable spinning without stalling on carpet, while also being slow enough to preserve enough ORB features for post-spin relocalization.

---

## Align to Object (real mode only)

After docking at a standoff position, `_run_align_sequence()` runs automatically. In `--mode test` it prints `[align] test mode — skipping align sequence` and returns `True`.

### 1. `search_for_object()`

Slow left-rotation scan (`SEARCH_SPIN = 35`) until an HSV colour blob appears in the camera frame (via `ColorDetector`). Timeout: `SEARCH_TIMEOUT = 20.0 s`.

### 2. `align_to_object()`

Iterative nudge-and-verify loop:

- **Fresh detection only**: 3 frames flushed via `_detect_fresh()` before each measurement to discard stale ZMQ-buffered frames.
- **Proportional pulse duration**: small position error → short pulse (`~50 ms`), large error → longer pulse (`~180 ms`). Avoids oscillation.
- **Speed decay**: `SPIN_START = 45` → `SPIN_MIN = 35` at `-SPIN_DECAY = 1` per iteration. Transitions from coarse sweep to fine trim automatically.
- **Stuck recovery**: if absolute pixel error hasn't decreased by more than 3 px over 6 consecutive iterations, the car jogs forward 0.15 s then backward 0.10 s to break static wheel friction, then resets the decay counter.
- **Success criterion**: centred within `DEAD_ZONE = ±40 px` for `CONFIRM_FRAMES = 3` consecutive fresh frames.

### 3. `approach_object()`

Proportional forward approach driven by detected object area:

- Area grows as the car advances. Target: `area >= 15000 px²` (default).
- Speed and pulse duration scale proportionally from `MAX_FWD_SPEED = 50` / `MAX_PULSE = 0.20 s` (far) down to `MIN_FWD_SPEED = 35` / `MIN_PULSE = 0.08 s` (near).
- **ZUPT-style stuck detection**: if area hasn't grown by at least `ZUPT_MIN_GROWTH = 100 px²` over a `ZUPT_WINDOW = 5`-reading window, the car is stuck. Soft unstick: single boosted burst at `BOOST_SPEED = 60`. Hard unstick (after `MAX_STUCK = 3` soft attempts): backward jog + strong forward push.
- **Re-alignment**: if the object drifts more than `DEAD_ZONE = 40 px` off-centre mid-approach, the car stops and re-runs `align_to_object()` for up to 10 s before resuming.

---

## Calibration Tools (`tools/`)

### `anchor.py`

Run once at the **start of each localization session**, with the car placed on a known ArUco marker. Reads 30 averaged SLAM poses, computes the offset between the current SLAM origin and the known world position of that marker (`ANCHOR_MARKER = 4`), and applies the correction to all subsequent coordinate readings. This is necessary because ORB-SLAM3's map origin drifts between sessions — the anchor step re-grounds the coordinate frame.

### `calibrate.py` + `calibrate_cli.py`

`calibrate.py` is the **math library**: takes phone photos of ArUco wall markers (M0, M4, M6, M7), runs `solvePnP` to extract 3D positions, then solves a Nelder-Mead optimisation for the room wall positions constrained to the known room dimensions (400 cm × 800 cm). Outputs a 3×3 affine transform matrix mapping SLAM `(x, z)` → room `(X, Y)` in centimetres.

`calibrate_cli.py` is the **CLI wrapper**: accepts photo file paths, prints per-photo detection results, validates the transform error, and writes `data/calibration.json`. Calibration is rejected and unsaved if the validation error ≥ 40 cm. The browser-based equivalent is the `/calibrate` POST endpoint in `web/server.py`.

```bash
python3 tools/calibrate_cli.py photo1.jpg photo2.jpg photo3.jpg
```

### `test_transform_static.py` / `test_transform_live.py`

Validation scripts used to measure and verify calibration accuracy:

- **Static**: loads `calibration.json` and `data/MarkerPositions.txt`, runs each marker's SLAM coordinates through the transform, and compares to hardcoded expected room positions. No ZMQ or SLAM required.
- **Live**: subscribes to `ZMQ:5557`, applies the same `calibrate.py` transform in real time, and prints per-pose room coordinates with in-bounds checks. Run while walking the car around the room to validate coverage.

The current calibration in `data/calibration.json` achieves a **33 cm mean validation error** across the four wall markers — sufficient for L-path navigation given the 0.30 m standoff threshold and the align-to-object final approach.

### `spin_debug.py` (referenced in `docs/SPIN_PROTOCOL.md`)

Isolated spin test harness used to tune `SPIN_SPEED` and `POST_SPIN_HOLD_SECS` without running the full navigation stack. Directly comparable to running `test_imu_nav.py` but with full SLAM integration — allows measurement of actual pose drift during and after a spin.

---

## File Structure

```
navigator.py          Unified navigator — --mode test (mock) / --mode real (hardware)
web/server.py         FastAPI backend, 7 endpoints: /, /state, /stations, /map_hull,
                      /calibration, /calibrate, /go, /stop
web/index.html        Dark-theme canvas dashboard: live map, phase tracker, calibration tab
stations.json         Calibrated station positions in SLAM coordinates (metres),
                      with orientation and standoff notes
tools/
  anchor.py           Session startup: SLAM origin correction from known marker
  calibrate.py        Math: ArUco photo → SLAM→room affine transform
  calibrate_cli.py    CLI wrapper for calibrate.py
  test_imu_live.py    Live IMU heading stream viewer (drift measurement)
  test_imu_nav.py     Interactive 90° spin tester with IMU feedback
  test_transform_static.py   Offline transform validation with known points
  test_transform_live.py     Live transform validation with moving car
  color_detect.py     HSV blob detector (ZMQ camera stream)
  align_to_object.py  Standalone align-to-object CLI (search/align/approach)
```

---

## API Endpoints (`web/server.py`)

| Method | Path | Description |
|---|---|---|
| `GET` | `/` | Serve `index.html` dashboard |
| `GET` | `/state` | Current position, heading, phase, docked station, path info |
| `GET` | `/stations` | All stations from `stations.json` |
| `GET` | `/map_hull` | Convex hull of `data/MapPoints.txt` (SLAM map boundary) |
| `GET` | `/calibration` | Contents of `data/calibration.json` |
| `POST` | `/calibrate` | Upload photos → compute + save new calibration |
| `POST` | `/go` | `{"target": "station_1"}` → start navigation in background thread |
| `POST` | `/stop` | Immediate motor stop, set phase to idle |

The `/go` endpoint is non-blocking: it starts `Navigator.Maps_to()` in a daemon thread and returns `{"status": "started"}` immediately. Poll `/state` to track progress.
