# Monitoring

Telemetry, storage, anomaly detection and a live dashboard for the autonomous car, built on MQTT (Mosquitto), InfluxDB 2 and a FastAPI + React dashboard.

> **Status:** the stack currently runs against **`mock/simulator.py` only**. Nothing in `navigation/`, `pi/` or `slam/` publishes to MQTT yet, and the ZMQ → MQTT bridge described in the project report is not written. The OPC-UA server is also not implemented yet: only its address-space mapping (`opcua_server/nodes.py`) exists. See [Known limits](#known-limits).

![Monitoring dashboard](../docs/media/monitoring_dashboard.png)

---

## Data flow

```
 mock/simulator.py ──MQTT──► Mosquitto :1883 ◄──MQTT── anomaly/detector.py
  (30 Hz pose, 50 Hz IMU,      │    │                     ▲   │ publishes car/anomaly/* on state change
   motors/phase on change)     │    │                     │   │ + writes measurement "car/anomaly"
                               │    ▼                     │   ▼
                               │  influx/writer.py ──► InfluxDB :8086  (org car_org, bucket car_telemetry)
                               │                          ▲  queried every 2 s by the detector (Flux)
                               ▼                          │
                  dashboard/backend/main.py :8000 ────────┘  /history/* queries
                  (FastAPI; subscribes car/#; WebSocket /ws; serves frontend/ as static files)
                               │
                               ▼
                  browser: index.html + App.js (React 18 + Recharts from CDN)
```

1. The **simulator** plays the navigator: it drives the same L-path between `start`, `station_1` and `station_2`, and publishes pose, IMU, motor and phase messages.
2. The **writer** stores every telemetry message in InfluxDB. The measurement name is the MQTT topic.
3. The **detector** queries InfluxDB every 2 s, runs four rules, and publishes an anomaly message only when a rule's state changes (found → cleared or the reverse).
4. The **dashboard backend** subscribes to `car/#`, pushes the latest message per topic to browsers over a WebSocket, and serves history from InfluxDB.

---

## Components

| Path | What it does |
|---|---|
| `docker-compose.yml` | Mosquitto (`eclipse-mosquitto:2`) and InfluxDB (`influxdb:2`, setup mode: org `car_org`, bucket `car_telemetry`, user `admin`). Credentials come from `.env`. |
| `mosquitto/config/mosquitto.conf` | `listener 1883`, `allow_anonymous true`, persistence in `mosquitto/data/`, log to `mosquitto/log/` and stdout. |
| `launch.py` | Runs `docker-compose up -d`, loads `.env`, then opens one terminal window per script: simulator, writer, detector, backend. Tries kitty, alacritty, gnome-terminal, konsole, xfce4-terminal, terminator, xterm. |
| `mock/simulator.py` | Car simulator with 7 injectable anomalies and runtime controls over `car/mock/*`. |
| `mock/inject.py` | CLI: `python3 mock/inject.py <anomaly> on|off` or `python3 mock/inject.py list`. |
| `mock/stress_test.py` | 18 scenario tests against a running simulator, at 4× speed; writes `stress_test_report.txt`. |
| `influx/writer.py` | MQTT → InfluxDB writer for the 4 telemetry topics. |
| `anomaly/detector.py` | Rule-based anomaly detector (Flux queries on InfluxDB). |
| `anomaly/peek.py` | Prints recent rows: `python3 anomaly/peek.py car/motors w a s d [window_seconds]`. |
| `anomaly/drift_test.py` | Resets the simulator, injects `imu_static_drift`, prints heading and alert state every second. |
| `dashboard/backend/main.py` | FastAPI app on port 8000: WebSocket, history, mock-control endpoints, static frontend. |
| `dashboard/frontend/` | `index.html` + `App.js` (compiled from `App.jsx` with Babel). `App.html` is an older single-file version that compiles JSX in the browser. |
| `opcua_server/nodes.py` | OPC-UA address-space factory (asyncua). `server.py`, `run.py` and `generate_certs.py` are empty. |

---

## How to run

### 1. Requirements

- Docker with Compose
- Python 3 with: `pip install paho-mqtt influxdb-client fastapi 'uvicorn[standard]'`
  (there is no requirements file; tested with Python 3.14, paho-mqtt 2.1.0, influxdb-client 1.50.0, fastapi 0.159.1, uvicorn 0.54.0). Plain `uvicorn` without `[standard]` has no WebSocket support: `/ws` returns 404 and the dashboard gets no live data.
- `mosquitto_pub` / `mosquitto_sub` (optional, for manual testing)

### 2. Credentials

```bash
cd monitoring
cp .env.example .env      # then set INFLUXDB_ADMIN_PASSWORD and INFLUXDB_TOKEN
```

`.env` is gitignored. `docker compose` reads it automatically and refuses to start if either value is missing. `launch.py` passes it to the Python scripts. When you run a script by hand, export it first:

```bash
set -a; source monitoring/.env; set +a
```

InfluxDB applies the password and token **only on first start**, when `influxdb/data/` is empty. If you change them later, the existing database keeps the old token. To start over with new values, stop the stack and delete `influxdb/data/` (this deletes all stored telemetry).

### 3. Start everything

```bash
cd monitoring
python3 launch.py
```

Open **http://localhost:8000**.

### Manual start (one terminal each)

```bash
cd monitoring
docker compose up -d
set -a; source .env; set +a
python3 mock/simulator.py
python3 influx/writer.py
python3 anomaly/detector.py
cd dashboard/backend && python3 main.py      # BACKEND_PORT env var overrides 8000
```

### Mock mode controls

Drive the simulator from the dashboard (station buttons, Reset to Start, simulation speed slider, anomaly toggles), from the CLI, or with raw MQTT:

```bash
python3 mock/inject.py motor_stall on
mosquitto_pub -h localhost -t car/mock/goto   -m '{"target":"station_1"}'
mosquitto_pub -h localhost -t car/mock/inject -m '{"anomaly":"tracking_loss","active":true}'
```

### Rebuild the frontend

`package.json` has no build script, and `node_modules/.bin` is missing, so call Babel directly:

```bash
cd dashboard/frontend
npm install
node node_modules/@babel/cli/bin/babel.js App.jsx -o App.js
```

The dashboard loads React 18, Recharts 2.12.7, Tailwind and Google Fonts from CDNs, so the browser needs internet access.

---

## Ports

| Port | Service | Notes |
|---|---|---|
| 1883 | Mosquitto MQTT | anonymous access |
| 9001 | Mosquitto | mapped in compose, but `mosquitto.conf` has no listener on it |
| 8086 | InfluxDB 2 HTTP API / UI | |
| 8000 | Dashboard backend + frontend | `BACKEND_PORT` env var |

---

## MQTT topics

### Telemetry (published by the simulator)

| Topic | Rate | Payload |
|---|---|---|
| `car/slam/pose` | 30 Hz | `{seq, ts, ok, x, z}` — `x`/`z` are `null` when `ok` is false |
| `car/imu` | 50 Hz | `{ts, heading_deg, true_heading_deg, raw_ax, raw_ay, moving, drift_elapsed_s, accumulated_drift_deg}` |
| `car/motors` | on change | `{w, a, s, d, total, inner, ts}` — same command shape the navigator sends to `car.py` |
| `car/nav/phase` | on change | `{phase, target_station, ts}` — phases: `departing`, `phase1`, `phase2`, `phase3`, `aligning`, `arrived`, `tracking_lost` |

`true_heading_deg`, `drift_elapsed_s` and `accumulated_drift_deg` are simulator-only fields, used to show injected IMU drift.

### Anomalies

| Topic | Published by | Payload |
|---|---|---|
| `car/anomaly/tracking_loss` | detector | `{type, severity: "CRIT", false_count, total_count, false_ratio, ts}` |
| `car/anomaly/motor_stall` | detector | `{type, severity: "CRIT", w_true_ratio, position_spread_mm, ts}` |
| `car/anomaly/imu_static_drift` | detector | `{type, severity: "CRIT", rate_first_half, rate_second_half, acceleration, heading_start, heading_end, ts}` |
| `car/anomaly/slam_low_feature` | detector | `{type, severity: "WARN", msg_count, rate_hz, ts}` |
| `car/anomaly/position_jump` | simulator | `{type, severity: "CRIT", jump_x, jump_z, magnitude, ts}` |

When a detector rule stops firing, it publishes `{type, severity: "CLEARED", ts}` on the same topic.

### Mock control (subscribed by the simulator)

| Topic | Payload |
|---|---|
| `car/mock/goto` | `{"target": "start" \| "station_1" \| "station_2"}` |
| `car/mock/reset` | `{"station": "start"}` — teleport and dock |
| `car/mock/inject` | `{"anomaly": "<name>", "active": true\|false}` |
| `car/mock/sim_speed` | `{"speed": 0.25 … 4.0}` (clamped) |
| `car/mock/motor_stall_params` | `{"severity": 0.0 … 1.0}` — 1.0 freezes position, below 1.0 the car arcs sideways |
| `car/mock/slam_params` | `{"decay": float, "recover": float}` — per-tick SLAM rate factors for `slam_low_feature` |
| `car/mock/position_jump_params` | `{"probability": 0.0 … 1.0}` |
| `car/mock/imu_drift_reset` | `{}` |

The simulator also publishes `car/mock/motor_stall_arc` `{arc_side: -1 | 1 | null}` for the dashboard.

---

## InfluxDB schema

- **Telemetry:** one measurement per topic (`car/slam/pose`, `car/imu`, `car/motors`, `car/nav/phase`). The timestamp comes from the payload `ts`. The writer stores only these fields:
  - pose: `seq`, `ok`, `x`, `z` (`x`/`z` are skipped when `null`)
  - IMU: `heading_deg`, `raw_ax`, `raw_ay`, `moving`
  - motors: `w`, `a`, `s`, `d`, `total`, `inner`
  - phase: `phase`, `target_station`
- **Anomalies:** measurement `car/anomaly`, tags `type` and `severity`, every other payload key as a field. The detector writes these directly; the writer does not subscribe to `car/anomaly/*`.

---

## Anomaly detection

`anomaly/detector.py` runs four checks every `CHECK_INTERVAL_S = 2` seconds, each over a sliding window queried from InfluxDB:

| Anomaly | Window | Fires when | Severity |
|---|---|---|---|
| `tracking_loss` | 5 s | more than 60% of `car/slam/pose` rows have `ok = false` | CRIT |
| `motor_stall` | 6 s | `w` (forward) is true in at least 70% of motor samples, **and** the pose spread (x range + z range) is under 4 mm | CRIT |
| `imu_static_drift` | 30 s | no motor command active and `moving` never true in the window, **and** the heading slope of the second half is above 0.00005°/sample **and** larger than the first half × 1.01 | CRIT |
| `slam_low_feature` | 5 s | pose message rate below 22 Hz | WARN |

Details:

- **Sparse motor topic:** `car/motors` is published only on change, so the motor checks carry the last command from before the window forward (looking back up to 30 min).
- **Heading unwrap:** headings are unwrapped across 0/360 before a least-squares slope is fitted to each half of the window. Drift is flagged only if it is significant *and* growing.
- **No arc-stall warning:** a WARN level for motor stall (a car arcing instead of driving straight) is left out on purpose. With one gyro heading source, arc-turning can't be told apart from IMU drift while the motors run.
- **Placeholder thresholds:** the `imu_static_drift` thresholds are marked TODO in the code. They need tuning with `peek.py` data from the real car.

Anomalies the simulator can inject but the detector does not detect: `position_jump` (the simulator reports it itself), `phase_timeout`, `trajectory_drift`.

![Anomalies on the dashboard](../docs/media/monitoring_anomalies.png)

---

## Dashboard

The backend (`dashboard/backend/main.py`) exposes:

| Method | Path | Purpose |
|---|---|---|
| WS | `/ws` | JSON snapshot of the latest message per topic on every MQTT message; anomaly messages are also sent as `{"_anomaly": true, "entry": …}` |
| GET | `/history/slam`, `/history/imu`, `/history/motors`, `/history/phase` | InfluxDB rows, `?minutes=1…1440` (default 10) |
| GET | `/anomalies` · POST `/anomalies/clear` | Latest entry per anomaly type |
| GET | `/live/slam_rate` | Pose msgs/s over the last 2 s: `ok` ≥ 20, `warn` ≥ 10, else `critical` |
| GET | `/live/motor_stall`, `/live/sim_speed`, `/live/position_jump`, `/live` | Current mock state |
| POST | `/api/inject`, `/api/goto`, `/api/publish`, `/sim/reset` | Publish to the `car/mock/*` topics (`/api/publish` accepts any topic) |
| POST | `/control/sim_speed`, `/control/position_jump_params`, `/control/imu_drift_reset` | Mock controls |
| GET | `/` | Static frontend |

The frontend shows:

- a status bar (SLAM tracking, phase, motor state, WebSocket)
- a navigation map with stations, live position and path, and goto / reset buttons
- a phase timeline, IMU heading chart, motor PWM chart (total / inner) and SLAM publish rate
- an anomaly feed
- a side panel with the simulation speed slider and anomaly toggles (Tracking Loss, Motor Stall, Position Jump, IMU Static Drift, Low Feature Room)

The frontend expects the backend at `http://localhost:8000` (`API_BASE` in `App.jsx`).

---

## OPC-UA (not implemented yet)

`opcua_server/nodes.py` defines the address space that the OPC-UA server is meant to expose. There is no server, endpoint or port yet. Planned browse paths, all writable and in the registered namespace:

| Folder | Variables | Source topic |
|---|---|---|
| `Objects/Pose` | `seq`, `ts`, `x`, `z` (Double), `ok` (Boolean) | `car/slam/pose` |
| `Objects/IMU` | `ts`, `heading_deg`, `true_heading_deg`, `raw_ax`, `raw_ay`, `drift_elapsed_s`, `accumulated_drift_deg` (Double), `moving` (Boolean) | `car/imu` |
| `Objects/Phase` | `phase`, `target_station` (String), `ts` (Double) | `car/nav/phase` |
| `Objects/Motors` | `w`, `a`, `s`, `d` (Boolean), `total`, `inner`, `ts` (Double) | `car/motors` |
| `Objects/Anomalies` | `TrackingLoss`, `MotorStall`, `ImuStaticDrift`, `SlamLowFeature` (Boolean: true unless the last severity was `CLEARED`) | `car/anomaly/*` |

---

## Stress test

`mock/stress_test.py` against the simulator at 4× speed. Last report (`mock/stress_test_report.txt`, 2026-06-19): **18 / 18 passed.**

| Test | Duration | Evidence |
|---|---|---|
| Route start → station_1 → station_2 → start | 39.9 s | all 3 legs arrived |
| Route start → station_2 → station_1 → start | 37.1 s | all 3 legs arrived |
| Baseline, no anomalies | 23.6 s | all phases seen |
| `tracking_loss` | 16.8 s | 503 `ok=false` poses, 2 `tracking_lost` phase events |
| `motor_stall` severity 1.0 | 19.0 s | x spread 0.0000 m |
| `motor_stall` severity 0.5 | 19.0 s | arc drift, total spread 1.2837 m |
| `position_jump` | 75.0 s | 0.1424 m jump, resets on re-inject |
| `phase_timeout` | 24.2 s | completed, slower |
| `imu_static_drift` | 42.4 s | 0.313° accumulated over 40 s |
| `trajectory_drift` | 18.8 s | max lateral deviation 0.0200 m |
| `slam_low_feature` | 12.2 s | average gap 13.0 ms → 17.3 ms, recovery 8.7 ms |
| tracking_loss + motor_stall | 27.1 s | both fired |
| imu_drift suppressed by motor_stall | 16.6 s | max drift 0.108° |
| trajectory_drift + motor_stall | 27.1 s | no crash |
| All 7 anomalies at once | 27.8 s | 5148 messages, simulator alive |
| 10 rapid on/off toggles | 12.7 s | stable |
| `position_jump` reset × 3 | 113.3 s | `[2, 2, 2]` |
| sim_speed 0.25× / 4× | 12.7 s | 38 / 574 SLAM messages in 5 s |

**This report predates the current simulator.** It counts per-tick `car/anomaly/motor_stall` messages from the simulator (1591 events), which the simulator no longer publishes; that topic now comes from the detector, on state change only. Re-run it with the detector running before quoting these numbers.

Run it with the broker and simulator up:

```bash
cd monitoring/mock && python3 stress_test.py
```

---

## Known limits

- **No real-car input.** There is no ZMQ → MQTT bridge, so the stack only sees simulator data.
- **No OPC-UA server** (see above).
- **Detector coverage:** 4 of the 6 failure modes in the report have detectors; `phase_timeout` and `trajectory_drift` have none.
- **`slam_low_feature` false positives:**
  - It fires right after startup, while the 5 s window is still filling.
  - It fires whenever `tracking_loss` is active, because the simulator publishes only about 30% of poses while tracking is lost.
- **`imu_static_drift` reacts slowly.** In a test at 4× simulation speed it took about 90 s real time to fire, and its thresholds are untuned placeholders.
- **Simulator arc mode:** in motor-stall mode with severity below 1.0, the car keeps moving but `car/imu` reports `moving: false`.
- **`mock/inject.py` gaps:**
  - It doesn't list `slam_low_feature`, though the simulator accepts it.
  - Run with no arguments, it prints `None` instead of usage.
- **`drift_test.py` speed:** it requests 50× speed; the simulator clamps it to 4×.
- **`"None"` strings in InfluxDB:** the writer stores a `null` `target_station` as the string `"None"`.
- **Deprecated MQTT API:** every script uses paho's deprecated callback API v1 (`mqtt.Client(client_id=...)`), which prints a `DeprecationWarning` on paho-mqtt 2.x.
- **No security:**
  - The MQTT broker is anonymous.
  - The backend allows CORS from any origin.
  - `/api/publish` publishes to any topic.
  - Keep the stack on localhost or a trusted network.
- **Single-machine setup:** hosts are hardcoded to `localhost` in every script and in the frontend.
- **`launch.py` needs a desktop:** it calls the v1 `docker-compose` command and needs a graphical terminal emulator.
- **Old credentials in git history:** earlier commits contain the InfluxDB credentials that now live in `.env` (`adminpass` / `super-token-123`). Treat those values as public and don't reuse them outside local development.
