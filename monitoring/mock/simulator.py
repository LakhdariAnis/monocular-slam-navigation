import json
import math
import random
import threading
import time

import paho.mqtt.client as mqtt

BROKER_HOST = "localhost"
BROKER_PORT = 1883

SLAM_HZ = 30
IMU_HZ  = 50
SLAM_DT = 1.0 / SLAM_HZ
IMU_DT  = 1.0 / IMU_HZ

FORWARD_SPEED       = 45
SLOW_SPEED          = 40
SPIN_SPEED          = 40
SEARCH_SPIN         = 35
SPIN_START          = 45
SPIN_MIN            = 35
SPIN_DECAY          = 1       # per iteration (not used directly in speed calc, kept for reference)
WAYPOINT_THRESHOLD  = 0.02   # elbow arrival (m)
STATION_THRESHOLD   = 0.015  # standoff arrival (m)
SPIN_TOLERANCE      = 5      # degrees
DEPART_DIST         = 0.30   # reverse distance (m)

FWD_FAST_RATE   = 0.008    # FORWARD_SPEED=45 total
FWD_SLOW_RATE   = 0.007    # SLOW_SPEED=40
REVERSE_RATE    = 0.008    # same as forward (FORWARD_SPEED used for depart)

SPIN_RATE_DEG   = 2.5      # SPIN_SPEED=40, imu@50Hz
SEARCH_SPIN_RATE = 2.2     # SEARCH_SPIN=35

POSITION_JUMP_MAGNITUDE  = 2.0     # metres — single-tick jump
DRIFT_RATE_DEG_PER_SEC   = 3.0     # heading drift during arrived/departing
TRAJECTORY_BEND_PER_SEC  = 0.015   # extra lateral shift per second during drives
PHASE_TIMEOUT_MULTIPLIER = 4.0     # how much longer phase_timeout makes transitions

STATIONS = {
    "start": {
        "x": -0.1155, "z": -0.2249,
        "orientation": "-Z Wall",
        "standoff": (-0.1155, 0.0551),
    },
    "station_1": {
        "x":  0.6900,  "z": 0.5521,
        "orientation": "+X Wall",
        "standoff": (0.3900, 0.5521),
    },
    "station_2": {
        "x": -0.0303,  "z": 1.5225,
        "orientation": "+Z Wall",
        "standoff": (-0.0303, 1.2225),
    },
}

ROUTE = [
    ("start",     "station_1"),
    ("station_1", "station_2"),
    ("station_2", "start"),
]

ARRIVED_HEADING = {
    "station_1": 90.0,
    "station_2":  0.0,
    "start":    180.0,
}

ALIGN_TARGET_AREA = 15000
ALIGN_INITIAL_AREA = 1000


_lock = threading.Lock()
_anomalies = {
    "tracking_loss":    False,
    "motor_stall":      False,
    "position_jump":    False,
    "phase_timeout":    False,
    "imu_static_drift": False,
    "trajectory_drift": False,
}
_position_jump_fired = False


def get_anomaly(key: str) -> bool:
    with _lock:
        return _anomalies.get(key, False)


def set_anomaly(key: str, value: bool):
    with _lock:
        if key in _anomalies:
            _anomalies[key] = value
            print(f"[INJECT] {key} → {'ON' if value else 'OFF'}")
        else:
            print(f"[INJECT] unknown anomaly key: {key}")


def _dist(x1, z1, x2, z2):
    return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)


def _bearing(cx, cz, tx, tz):
    return math.degrees(math.atan2(tx - cx, tz - cz))


def _angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def _compute_elbow(from_name, to_name):
    src = STATIONS[from_name]
    dst = STATIONS[to_name]
    orientation = dst["orientation"]

    if from_name == "start":
        cx, cz = src["x"], src["z"]
    else:
        cx, cz = src["standoff"]

    sx, sz = dst["standoff"]

    if orientation in ("+X Wall", "-X Wall"):
        elbow_x, elbow_z = cx, sz
    else:
        elbow_x, elbow_z = sx, cz

    return elbow_x, elbow_z, sx, sz


client = mqtt.Client(client_id="mock_simulator")


def on_connect(c, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT] connected to {BROKER_HOST}:{BROKER_PORT}")
        c.subscribe("car/mock/inject")
        print("[MQTT] subscribed to car/mock/inject")
    else:
        print(f"[MQTT] connection failed rc={rc}")


def on_message(c, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        anomaly = payload.get("anomaly", "")
        active  = bool(payload.get("active", False))
        set_anomaly(anomaly, active)
    except Exception as e:
        print(f"[INJECT] bad payload: {e}")


client.on_connect = on_connect
client.on_message = on_message


def publish(topic: str, payload: dict):
    client.publish(topic, json.dumps(payload), qos=0)


class State:
    def __init__(self):
        self.x           = STATIONS["start"]["x"]
        self.z           = STATIONS["start"]["z"]
        self.heading_deg = 180.0

        self.phase          = "arrived"
        self.target_station = "station_1"

        self.last_motor_cmd = None

        self.motion_intent = "stopped"
        self.forward_dir_x = 0.0
        self.forward_dir_z = 0.0
        self.forward_speed = FORWARD_SPEED

        self.align_area    = 0

        self.lock = threading.Lock()


state = State()


def _publish_motor(w, a, s, d, total, inner):
    now = time.time()
    cmd = {"w": w, "a": a, "s": s, "d": d,
           "total": total, "inner": inner, "ts": now}
    with state.lock:
        if cmd == state.last_motor_cmd:
            return
        state.last_motor_cmd = cmd
    publish("car/motors", cmd)


def _pub_stop():
    _publish_motor(False, False, False, False, 0, 0)


def _pub_forward(speed=FORWARD_SPEED):
    inner = int(speed * 0.75)
    _publish_motor(True, False, False, False, speed, inner)


def _pub_reverse():
    speed = FORWARD_SPEED
    inner = int(speed * 0.75)
    _publish_motor(False, False, True, False, speed, inner)


def _pub_spin(left=True, speed=SPIN_SPEED):
    if left:
        _publish_motor(False, True, False, False, speed, speed)
    else:
        _publish_motor(False, False, False, True, speed, speed)


def _set_phase(phase: str, target_station: str = None):
    now = time.time()
    with state.lock:
        state.phase = phase
        if target_station is not None:
            state.target_station = target_station
        ts_val = state.target_station
    payload = {
        "phase":          phase,
        "target_station": ts_val,
        "ts":             now,
    }
    publish("car/nav/phase", payload)
    print(f"[PHASE] → {phase}  (station {ts_val})")


def slam_loop():
    global _position_jump_fired
    seq = 0

    while True:
        t0  = time.monotonic()
        now = time.time()
        seq += 1

        tracking_ok = True

        if get_anomaly("tracking_loss"):
            tracking_ok = False

        if get_anomaly("position_jump") and not _position_jump_fired:
            with state.lock:
                state.x += POSITION_JUMP_MAGNITUDE
                state.z += POSITION_JUMP_MAGNITUDE
            _position_jump_fired = True
        elif not get_anomaly("position_jump"):
            _position_jump_fired = False

        if tracking_ok and not get_anomaly("motor_stall"):
            with state.lock:
                intent = state.motion_intent
                fdx    = state.forward_dir_x
                fdz    = state.forward_dir_z
                fspd   = state.forward_speed

            if intent == "forward":
                rate = FWD_FAST_RATE if fspd >= FORWARD_SPEED else FWD_SLOW_RATE
                dx = fdx * rate + random.gauss(0, 0.0005)
                dz = fdz * rate + random.gauss(0, 0.0005)
                if get_anomaly("trajectory_drift"):
                    perp_x = -fdz
                    perp_z =  fdx
                    dx += perp_x * TRAJECTORY_BEND_PER_SEC * SLAM_DT
                    dz += perp_z * TRAJECTORY_BEND_PER_SEC * SLAM_DT
                with state.lock:
                    state.x = round(state.x + dx, 4)
                    state.z = round(state.z + dz, 4)

            elif intent == "reverse":
                dx = -fdx * REVERSE_RATE + random.gauss(0, 0.0005)
                dz = -fdz * REVERSE_RATE + random.gauss(0, 0.0005)
                with state.lock:
                    state.x = round(state.x + dx, 4)
                    state.z = round(state.z + dz, 4)

        with state.lock:
            x = state.x
            z = state.z

        if tracking_ok:
            payload = {
                "seq": seq,
                "ts":  round(now, 6),
                "ok":  True,
                "x":   x,
                "z":   z,
            }
            publish("car/slam/pose", payload)
        else:
            if random.random() >= 0.7:
                payload = {
                    "seq": seq,
                    "ts":  round(now, 6),
                    "ok":  False,
                    "x":   None,
                    "z":   None,
                }
                publish("car/slam/pose", payload)

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, SLAM_DT - elapsed))


def imu_loop():
    while True:
        t0  = time.monotonic()
        now = time.time()

        with state.lock:
            intent = state.motion_intent
            phase  = state.phase

        if intent == "spin":
            with state.lock:
                spin_sign = state.forward_dir_x   # +1 → CCW (left), -1 → CW (right)
            delta = spin_sign * SPIN_RATE_DEG + random.gauss(0, 0.05)
            with state.lock:
                state.heading_deg = (state.heading_deg + delta) % 360
        elif get_anomaly("imu_static_drift") and phase in ("arrived", "departing"):
            with state.lock:
                state.heading_deg = (state.heading_deg + DRIFT_RATE_DEG_PER_SEC * IMU_DT) % 360
        else:
            with state.lock:
                state.heading_deg = (state.heading_deg + random.gauss(0, 0.01)) % 360

        moving = intent in ("forward", "reverse", "spin")
        if get_anomaly("motor_stall"):
            moving = False

        if moving:
            raw_ax = round(random.gauss(0.05, 0.008), 5)
            raw_ay = round(random.gauss(0.02, 0.005), 5)
        else:
            raw_ax = round(random.gauss(0.0, 0.001), 5)
            raw_ay = round(random.gauss(0.0, 0.001), 5)

        with state.lock:
            heading_out = round(state.heading_deg, 2)

        payload = {
            "ts":          round(now, 6),
            "heading_deg": heading_out,
            "raw_ax":      raw_ax,
            "raw_ay":      raw_ay,
            "moving":      moving,
        }
        publish("car/imu", payload)

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, IMU_DT - elapsed))


def _sim_spin(target_bearing_deg, spin_speed=SPIN_SPEED):
    target_bearing_deg = target_bearing_deg % 360

    with state.lock:
        current = state.heading_deg

    diff = _angle_diff(target_bearing_deg, current)
    turn_right = diff > 0

    with state.lock:
        state.motion_intent  = "spin"
        state.forward_dir_x  = -1.0 if turn_right else +1.0
        state.forward_dir_z  = 0.0

    _pub_spin(left=not turn_right, speed=spin_speed)

    poll_interval = 0.02
    if get_anomaly("phase_timeout"):
        poll_interval *= PHASE_TIMEOUT_MULTIPLIER

    while True:
        time.sleep(poll_interval)
        with state.lock:
            current = state.heading_deg
        df = _angle_diff(target_bearing_deg, current)
        if abs(df) <= SPIN_TOLERANCE:
            break

    with state.lock:
        state.motion_intent = "stopped"
        state.heading_deg   = target_bearing_deg
    _pub_stop()
    time.sleep(0.2)


def _sim_drive(target_x, target_z, threshold,
               heading_bearing_deg,
               reverse=False, mid_switch=True):
    rad = math.radians(heading_bearing_deg)
    dir_x = math.sin(rad)
    dir_z = math.cos(rad)

    if reverse:
        dir_x = -dir_x
        dir_z = -dir_z

    with state.lock:
        state.forward_dir_x = dir_x if not reverse else -dir_x
        state.forward_dir_z = dir_z if not reverse else -dir_z
        state.motion_intent  = "reverse" if reverse else "forward"
        cx, cz = state.x, state.z

    start_dist = _dist(cx, cz, target_x, target_z)
    initial_d  = start_dist

    effective_threshold = threshold
    if get_anomaly("phase_timeout"):
        effective_threshold = threshold / PHASE_TIMEOUT_MULTIPLIER

    speed = FORWARD_SPEED
    if not reverse:
        _pub_forward(speed)
    else:
        _pub_reverse()

    poll_interval = 0.05

    while True:
        time.sleep(poll_interval)

        if get_anomaly("motor_stall"):
            # Position won't advance; just spin here waiting
            continue

        with state.lock:
            cx, cz = state.x, state.z

        d = _dist(cx, cz, target_x, target_z)

        if mid_switch and not reverse:
            in_approach = d < initial_d * 0.5
            new_speed = SLOW_SPEED if in_approach else FORWARD_SPEED
            if new_speed != speed:
                speed = new_speed
                with state.lock:
                    state.forward_speed = speed
                _pub_forward(speed)

        if d <= effective_threshold:
            break

    with state.lock:
        state.motion_intent  = "stopped"
        state.forward_speed  = FORWARD_SPEED
    _pub_stop()
    time.sleep(0.2)


def _sim_depart(from_station_name):
    info = STATIONS[from_station_name]
    orientation = info["orientation"]

    orient_vec = {
        "-X Wall": (+1,  0),
        "+X Wall": (-1,  0),
        "-Z Wall": ( 0, +1),
        "+Z Wall": ( 0, -1),
    }
    vx, vz = orient_vec[orientation]

    sx, sz = info["standoff"] if from_station_name != "start" else (info["x"], info["z"])

    clear_x = sx - vx * DEPART_DIST
    clear_z = sz - vz * DEPART_DIST

    bear = math.degrees(math.atan2(vx, vz)) % 360

    with state.lock:
        state.motion_intent  = "reverse"
        state.forward_dir_x  = vx
        state.forward_dir_z  = vz
        state.forward_speed  = FORWARD_SPEED

    _pub_reverse()

    effective_threshold = WAYPOINT_THRESHOLD
    if get_anomaly("phase_timeout"):
        effective_threshold = WAYPOINT_THRESHOLD / PHASE_TIMEOUT_MULTIPLIER

    poll_interval = 0.05
    while True:
        time.sleep(poll_interval)
        if get_anomaly("motor_stall"):
            continue
        with state.lock:
            cx, cz = state.x, state.z
        d = _dist(cx, cz, clear_x, clear_z)
        if d <= effective_threshold:
            break

    with state.lock:
        state.motion_intent = "stopped"
    _pub_stop()
    time.sleep(0.2)


def _sim_search_for_object():
    target_offset = random.uniform(20, 90)
    with state.lock:
        current_hdg = state.heading_deg
    target_hdg = (current_hdg + target_offset) % 360

    with state.lock:
        state.motion_intent = "spin"
        state.forward_dir_x = +1.0   # CCW / left
        state.forward_dir_z = 0.0
    _pub_spin(left=True, speed=SEARCH_SPIN)

    poll = 0.02
    if get_anomaly("phase_timeout"):
        poll *= PHASE_TIMEOUT_MULTIPLIER

    while True:
        time.sleep(poll)
        with state.lock:
            hdg = state.heading_deg
        diff = _angle_diff(target_hdg, hdg)
        if abs(diff) <= SPIN_TOLERANCE:
            break

    with state.lock:
        state.motion_intent = "stopped"
        state.align_area    = ALIGN_INITIAL_AREA
    _pub_stop()
    time.sleep(0.2)


def _sim_align_to_object():
    target_offset = random.uniform(5, 25)
    with state.lock:
        current_hdg = state.heading_deg
    target_hdg = (current_hdg + target_offset) % 360

    iteration = 0
    poll = 0.02
    if get_anomaly("phase_timeout"):
        poll *= PHASE_TIMEOUT_MULTIPLIER

    with state.lock:
        state.motion_intent = "spin"
        state.forward_dir_x = +1.0
        state.forward_dir_z = 0.0

    while True:
        spin_speed = max(SPIN_MIN, SPIN_START - iteration * SPIN_DECAY)
        _pub_spin(left=True, speed=spin_speed)
        time.sleep(poll)
        iteration += 1
        with state.lock:
            hdg = state.heading_deg
        diff = _angle_diff(target_hdg, hdg)
        if abs(diff) <= SPIN_TOLERANCE:
            break

    with state.lock:
        state.motion_intent = "stopped"
    _pub_stop()
    time.sleep(0.2)


def _sim_approach_object():
    MAX_FWD_SPEED = 50
    MIN_FWD_SPEED = 35

    with state.lock:
        hdg = state.heading_deg
    rad = math.radians(hdg)
    dir_x = math.sin(rad)
    dir_z = math.cos(rad)

    with state.lock:
        state.motion_intent  = "forward"
        state.forward_dir_x  = dir_x
        state.forward_dir_z  = dir_z
        state.forward_speed  = MAX_FWD_SPEED
        area = state.align_area

    poll = 0.08
    if get_anomaly("phase_timeout"):
        poll *= PHASE_TIMEOUT_MULTIPLIER

    while True:
        with state.lock:
            area = state.align_area

        if area >= ALIGN_TARGET_AREA:
            break

        progress = min(area / ALIGN_TARGET_AREA, 1.0)
        speed = int(MAX_FWD_SPEED - progress * (MAX_FWD_SPEED - MIN_FWD_SPEED))
        _pub_forward(speed)

        time.sleep(poll)
        if not get_anomaly("motor_stall"):
            growth = random.randint(300, 700) * (speed / MAX_FWD_SPEED)
            with state.lock:
                state.align_area = min(int(state.align_area + growth), ALIGN_TARGET_AREA + 1000)

    with state.lock:
        state.motion_intent = "stopped"
    _pub_stop()
    time.sleep(0.2)


def phase_loop():
    with state.lock:
        state.x           = STATIONS["start"]["x"]
        state.z           = STATIONS["start"]["z"]
        state.heading_deg = 180.0   # facing away from -Z wall (south)
        state.motion_intent = "stopped"

    _set_phase("arrived", target_station="station_1")
    _pub_stop()

    leg_index   = 0
    first_leg   = True   # skip DEPART on very first leg

    while True:
        from_name, to_name = ROUTE[leg_index % len(ROUTE)]

        print(f"\n[SIM] ═══ Leg: {from_name} → {to_name} ═══")

        elbow_x, elbow_z, standoff_x, standoff_z = _compute_elbow(from_name, to_name)

        with state.lock:
            cx, cz = state.x, state.z

        bear_to_elbow    = _bearing(cx, cz, elbow_x, elbow_z)
        bear_to_standoff = _bearing(elbow_x, elbow_z, standoff_x, standoff_z)

        print(f"  from=({cx:+.4f},{cz:+.4f})  elbow=({elbow_x:+.4f},{elbow_z:+.4f})"
              f"  standoff=({standoff_x:+.4f},{standoff_z:+.4f})")
        print(f"  bear_to_elbow={bear_to_elbow:.1f}°  bear_to_standoff={bear_to_standoff:.1f}°")

        if not first_leg:
            _set_phase("departing", target_station=to_name)
            _sim_depart(from_name)

            info = STATIONS[from_name]
            orient_vec = {
                "-X Wall": (+1,  0), "+X Wall": (-1,  0),
                "-Z Wall": ( 0, +1), "+Z Wall": ( 0, -1),
            }
            vx, vz = orient_vec[info["orientation"]]
            sx, sz = info["standoff"] if from_name != "start" else (info["x"], info["z"])
            with state.lock:
                state.x = round(sx - vx * DEPART_DIST, 4)
                state.z = round(sz - vz * DEPART_DIST, 4)
                cx, cz  = state.x, state.z

            elbow_x, elbow_z, standoff_x, standoff_z = _compute_elbow(from_name, to_name)
            bear_to_elbow    = _bearing(cx, cz, elbow_x, elbow_z)
            bear_to_standoff = _bearing(elbow_x, elbow_z, standoff_x, standoff_z)

        first_leg = False

        _set_phase("phase1", target_station=to_name)

        _pub_spin(left=(bear_to_elbow >= 0), speed=SPIN_SPEED)
        _sim_spin(bear_to_elbow, spin_speed=SPIN_SPEED)

        with state.lock:
            cx, cz = state.x, state.z

        _sim_drive(elbow_x, elbow_z, WAYPOINT_THRESHOLD,
                   bear_to_elbow, reverse=False, mid_switch=True)

        with state.lock:
            state.x = round(elbow_x, 4)
            state.z = round(elbow_z, 4)

        time.sleep(0.5)

        _set_phase("phase2", target_station=to_name)

        with state.lock:
            cx, cz = state.x, state.z
        bear_to_standoff = _bearing(cx, cz, standoff_x, standoff_z)

        _sim_spin(bear_to_standoff, spin_speed=SPIN_SPEED)

        time.sleep(0.5)

        _set_phase("phase3", target_station=to_name)

        with state.lock:
            cx, cz = state.x, state.z
        bear_to_standoff = _bearing(cx, cz, standoff_x, standoff_z)

        _sim_drive(standoff_x, standoff_z, STATION_THRESHOLD,
                   bear_to_standoff, reverse=False, mid_switch=True)

        with state.lock:
            state.x = round(standoff_x, 4)
            state.z = round(standoff_z, 4)

        _set_phase("aligning", target_station=to_name)

        with state.lock:
            state.align_area = ALIGN_INITIAL_AREA

        _sim_search_for_object()

        _sim_align_to_object()

        _sim_approach_object()

        _set_phase("arrived", target_station=to_name)
        _pub_stop()

        with state.lock:
            state.heading_deg = ARRIVED_HEADING.get(to_name, 0.0)
            state.motion_intent = "stopped"
            state.align_area    = 0

        print(f"[SIM] ✓ arrived at {to_name}")

        time.sleep(2.0)

        leg_index += 1


def main():
    print(f"[SIM] connecting to MQTT broker at {BROKER_HOST}:{BROKER_PORT} ...")
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    client.loop_start()

    time.sleep(1.0)

    print("[SIM] starting sensor loops ...")
    print("[SIM] publish  → car/slam/pose  @ ~30 Hz")
    print("[SIM] publish  → car/imu        @ 50 Hz")
    print("[SIM] publish  → car/nav/phase  @ phase change")
    print("[SIM] publish  → car/motors     @ command change")
    print("[SIM] subscribe→ car/mock/inject")
    print()
    print("[SIM] inject example:")
    print("  mosquitto_pub -h localhost -t car/mock/inject -m '{\"anomaly\":\"motor_stall\",\"active\":true}'")
    print()

    threading.Thread(target=slam_loop, daemon=True).start()
    threading.Thread(target=imu_loop,  daemon=True).start()

    phase_loop()   # blocks main thread


if __name__ == "__main__":
    main()