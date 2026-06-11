"""
navigator.py — Unified L-shape navigation (test + real modes)

════════════════════════════════════════════════════════════════
  Usage:
    python3 navigator.py --mode test   station_1   # mock sensors, fast steps
    python3 navigator.py --mode real   station_1   # real ZMQ sensors + Pi HTTP
    python3 navigator.py --list                    # show all stations
════════════════════════════════════════════════════════════════
"""

import json, math, time, sys, os, threading, argparse
import requests
from pathlib import Path as _Path
import sys as _sys
_sys.path.insert(0, str(_Path(__file__).parent / "tools"))
from color_detect import ColorDetector, STREAM_ADDR

# ── CONFIG ────────────────────────────────────────────────────────────────────
PI_HOST       = "YOUR_PI_IP"
PI_PORT       = 5000
SLAM_HOST     = "localhost"
SLAM_PORT     = 5557
IMU_HOST      = "YOUR_PI_IP"
IMU_PORT      = 5556
STATIONS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "stations.json")
CAR_URL       = f"http://{PI_HOST}:{PI_PORT}/drive"

# ── TUNING ────────────────────────────────────────────────────────────────────
# ── Align-to-object tuning ──
DEAD_ZONE        = 40
CONFIRM_FRAMES   = 3
SEARCH_TIMEOUT   = 20.0
ALIGN_TIMEOUT    = 30.0
SPIN_START       = 45
SPIN_MIN         = 35
SPIN_DECAY       = 1
UNSTICK_SPIN     = 60
UNSTICK_PULSE    = 0.12
SEARCH_SPIN      = 35

FORWARD_SPEED       = 45
SLOW_SPEED          = 40
SPIN_SPEED          = 40
WAYPOINT_THRESHOLD  = 0.02      # elbow arrival
STATION_THRESHOLD   = 0.015     # standoff arrival
ANGLE_THRESHOLD     = 12        # heading correction trigger
SPIN_TOLERANCE      = 5         # spin done when within N° of target
STALL_TIMEOUT       = 3.0       # seconds before stall recovery
STANDOFF_DIST       = 0.30      # metres in front of station marker
SLOW_DISTANCE       = 0.20      # switch to SLOW_SPEED within this distance
STALE_LIMIT         = 5         # identical SLAM frames before stale warning
STALE_TIMEOUT       = 3.0       # seconds to wait for fresh SLAM after stale
PRINT_INTERVAL      = 0.5       # seconds between drive-loop prints
PHASE_PAUSE         = 0.5       # seconds to pause between auto phases
POST_SPIN_HOLD_SECS = 1.0       # pause after spin stop (prevents rollback)
POST_SPIN_SLAM_WAIT = 0.5       # extra wait for SLAM to settle after spin
DEPART_DIST         = 0.30      # reverse distance when departing a station
STRAIGHT_THRESHOLD  = 0.03      # lateral offset < this → straight line

# ── MOCK TUNING ───────────────────────────────────────────────────────────────
_MOCK_MOVE_STEP = 0.006
_MOCK_SPIN_STEP = 1.5

# ── ORIENTATION → standoff direction vector ────────────────────────────────────
ORIENTATION_VECTOR = {
    "-X Wall": (+1,  0),
    "+X Wall": (-1,  0),
    "-Z Wall": ( 0, +1),
    "+Z Wall": ( 0, -1),
}

# ══════════════════════════════════════════════════════════════════════════════
#  MODULE-LEVEL STATE  (read by web/server.py)
# ══════════════════════════════════════════════════════════════════════════════
_mode         = "test"                        # "test" | "real", set at startup
_docked_at    = None                          # None | "station_name"
_mock_pose    = (-0.1155, -0.2249)            # current position
_mock_heading = 180.0                         # current heading degrees
_mock_lock    = threading.Lock()

_phase        = "idle"                        # current navigation phase
_running      = False                         # True while navigating
_path_info    = None                          # path dict for web frontend
_align_status = ""

_state_lock   = threading.Lock()

_nav_thread   = None                          # background navigation thread
_stop_event   = threading.Event()             # signal to abort navigation

_stations     = {}                            # loaded from stations.json


# ── TERMINAL HELPERS ──────────────────────────────────────────────────────────
_W = 60

def _sep(char="─"):
    print(char * _W)

def _banner(title: str):
    print()
    _sep("═")
    pad = (_W - len(title) - 2) // 2
    print(" " * pad + f" {title} ")
    _sep("═")

def _phase_banner(n: int, desc: str):
    print()
    _sep()
    print(f"  PHASE {n}  ─  {desc}")
    _sep()

def _ok(msg: str):
    print(f"  ✓  {msg}")

def _warn(msg: str):
    print(f"  !!  {msg}")

def _info(msg: str):
    print(f"      {msg}")


# ══════════════════════════════════════════════════════════════════════════════
#  SENSORS — real (ZMQ) vs test (mock)
# ══════════════════════════════════════════════════════════════════════════════

class _MockImuReader:
    """Test-mode IMU: reads from module-level _mock_heading."""
    def get(self):
        with _mock_lock:
            return _mock_heading

class _MockSlamReader:
    """Test-mode SLAM: reads from module-level _mock_pose."""
    def get(self):
        with _mock_lock:
            return _mock_pose


class _RealImuReader:
    """Real-mode IMU via ZMQ SUB (CONFLATE=1)."""
    def __init__(self, ctx):
        import zmq
        self._heading = None
        self._lock = threading.Lock()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.connect(f"tcp://{IMU_HOST}:{IMU_PORT}")
        self._sock = sock
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                data = json.loads(self._sock.recv_string())
                with self._lock:
                    self._heading = data["heading_deg"]
            except Exception:
                pass

    def get(self):
        with self._lock:
            return self._heading


class _RealSlamReader:
    """Real-mode SLAM via ZMQ SUB (CONFLATE=1)."""
    def __init__(self, ctx):
        import zmq
        self._pose = None
        self._lock = threading.Lock()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.connect(f"tcp://{SLAM_HOST}:{SLAM_PORT}")
        sock.setsockopt(zmq.RCVTIMEO, 2000)
        self._sock = sock
        time.sleep(1)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                data = json.loads(self._sock.recv_string())
                with self._lock:
                    self._pose = (data["x"], data["z"])
            except Exception:
                pass

    def get(self):
        with self._lock:
            return self._pose


# ══════════════════════════════════════════════════════════════════════════════
#  CAR COMMANDS — real (HTTP) vs test (mock print + sim)
# ══════════════════════════════════════════════════════════════════════════════

def _car_stop():
    if _mode == "real":
        import requests
        for _ in range(3):
            try:
                requests.post(CAR_URL,
                    json={"w": False, "a": False, "s": False, "d": False,
                          "total": 0, "inner": 0},
                    timeout=1.0)
            except Exception:
                pass
            time.sleep(0.05)
    else:
        print("  [mock] stop")

def _car_forward(speed: int = FORWARD_SPEED):
    global _mock_pose
    if _mode == "real":
        import requests
        try:
            requests.post(CAR_URL,
                json={"w": True,  "a": False, "s": False, "d": False,
                      "total": speed, "inner": int(speed * 0.75)},
                timeout=1.0)
        except Exception as e:
            _warn(f"[forward] {e}")
    else:
        with _mock_lock:
            heading_rad = math.radians(_mock_heading)
            dx = math.sin(heading_rad) * _MOCK_MOVE_STEP
            dz = math.cos(heading_rad) * _MOCK_MOVE_STEP
            x, z = _mock_pose
            _mock_pose = (x + dx, z + dz)

def _car_spin_left():
    global _mock_heading
    if _mode == "real":
        import requests
        try:
            requests.post(CAR_URL,
                json={"w": False, "a": True,  "s": False, "d": False,
                      "total": SPIN_SPEED, "inner": SPIN_SPEED},
                timeout=1.0)
        except Exception:
            pass
    else:
        with _mock_lock:
            _mock_heading = (_mock_heading - _MOCK_SPIN_STEP) % 360

def _car_spin_right():
    global _mock_heading
    if _mode == "real":
        import requests
        try:
            requests.post(CAR_URL,
                json={"w": False, "a": False, "s": False, "d": True,
                      "total": SPIN_SPEED, "inner": SPIN_SPEED},
                timeout=1.0)
        except Exception:
            pass
    else:
        with _mock_lock:
            _mock_heading = (_mock_heading + _MOCK_SPIN_STEP) % 360

def _car_reverse():
    global _mock_pose
    if _mode == "real":
        import requests
        try:
            requests.post(CAR_URL,
                json={"w": False, "a": False, "s": True,  "d": False,
                      "total": FORWARD_SPEED, "inner": int(FORWARD_SPEED * 0.75)},
                timeout=1.0)
        except Exception:
            pass
    else:
        with _mock_lock:
            heading_rad = math.radians(_mock_heading)
            dx = math.sin(heading_rad) * _MOCK_MOVE_STEP
            dz = math.cos(heading_rad) * _MOCK_MOVE_STEP
            x, z = _mock_pose
            _mock_pose = (x - dx, z - dz)


# ══════════════════════════════════════════════════════════════════════════════
#  GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════

def _dist(x1, z1, x2, z2):
    return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

def _bearing(cx, cz, tx, tz):
    """Compass bearing (degrees) from car to target in SLAM XZ space."""
    return math.degrees(math.atan2(tx - cx, tz - cz))

def _angle_diff(target, current):
    """Signed shortest angular difference: target − current, wrapped to ±180."""
    return (target - current + 180) % 360 - 180

def _cross2d(ax, az, bx, bz):
    """2D cross product in XZ plane. Positive → CCW (left). Negative → CW (right)."""
    return ax * bz - az * bx

def _lateral_offset(car_x, car_z, target_x, target_z, approach_vx, approach_vz):
    """Perpendicular offset from car to the approach axis."""
    dx = target_x - car_x
    dz = target_z - car_z
    perp_x = -approach_vz
    perp_z =  approach_vx
    return abs(dx * perp_x + dz * perp_z)

def _parse_approach(orientation):
    """Convert orientation string to approach direction vector."""
    if not orientation or len(orientation) < 2:
        return (0.0, 0.0)
    sign = +1.0 if orientation[0] == '+' else -1.0
    axis = orientation[1].upper()
    return (sign, 0.0) if axis == 'X' else (0.0, sign)


# ══════════════════════════════════════════════════════════════════════════════
#  MOTION PRIMITIVES
# ══════════════════════════════════════════════════════════════════════════════

def _get_pose(slam):
    """Get current pose; also update module-level _mock_pose/_mock_heading for web."""
    global _mock_pose
    pose = slam.get()
    if pose is not None and _mode == "real":
        with _mock_lock:
            _mock_pose = pose
    return pose

def _get_heading(imu):
    """Get current heading; also update module-level _mock_heading for web."""
    global _mock_heading
    hdg = imu.get()
    if hdg is not None and _mode == "real":
        with _mock_lock:
            _mock_heading = hdg
    return hdg


def _spin_to_bearing(target_bear, imu):
    global _mock_heading
    target_bear = target_bear % 360
    start_hdg = _get_heading(imu) or 0.0
    
    # Calculate initial shortest-path angular difference
    initial_df = (target_bear - start_hdg + 180) % 360 - 180
    turn_right = initial_df > 0  # In our CW system, positive diff means turn Right
    
    while not _stop_event.is_set():
        hdg = _get_heading(imu)
        if hdg is None:
            time.sleep(0.02)
            continue
        
        df = (target_bear - hdg + 180) % 360 - 180
        if abs(df) <= SPIN_TOLERANCE:
            _car_stop()
            if _mode == "test":
                with _mock_lock:
                    _mock_heading = target_bear
            time.sleep(0.2)

            # Post-spin hold — let the car settle before reading SLAM
            time.sleep(POST_SPIN_HOLD_SECS)

            # Post-spin SLAM wait — wait for a fresh pose
            if _mode == "real":
                t0 = time.time()
                while time.time() - t0 < POST_SPIN_SLAM_WAIT:
                    time.sleep(0.05)

            return
        
        _car_spin_right() if turn_right else _car_spin_left()
        time.sleep(0.02)


def _correct_heading(target_bear: float, imu):
    """Quick in-place heading correction during a drive leg."""
    global _mock_heading
    print(f"      [hdg] correcting → {target_bear:.1f}°")
    
    target_bear = target_bear % 360
    start_hdg = _get_heading(imu) or 0.0
    
    initial_df = (target_bear - start_hdg + 180) % 360 - 180
    turn_right = initial_df > 0
    
    while not _stop_event.is_set():
        hdg = _get_heading(imu)
        if hdg is None:
            time.sleep(0.02)
            continue
        
        df = (target_bear - hdg + 180) % 360 - 180
        if abs(df) <= SPIN_TOLERANCE:
            _car_stop()
            if _mode == "test":
                with _mock_lock:
                    _mock_heading = target_bear
            time.sleep(0.2)

            # Post-spin hold — let the car settle before reading SLAM
            time.sleep(POST_SPIN_HOLD_SECS)

            # Post-spin SLAM wait — wait for a fresh pose
            if _mode == "real":
                t0 = time.time()
                while time.time() - t0 < POST_SPIN_SLAM_WAIT:
                    time.sleep(0.05)

            return
            
        _car_spin_right() if turn_right else _car_spin_left()
        time.sleep(0.02)


def _drive_to(name: str, tx: float, tz: float, threshold: float,
              slam, imu, heading_correction: bool = True, reverse: bool = False):
    """Drive forward (or reverse) until within `threshold` metres of (tx, tz)."""
    global _mock_heading
    _info(f"target=({tx:+.3f}, {tz:+.3f})   threshold={threshold:.3f} m")


    stuck       = 0
    last_d      = 9999.0
    last_pose   = None
    stale_count = 0
    initial_d   = None
    last_print  = 0.0

    while not _stop_event.is_set():
        pose = _get_pose(slam)
        if pose is None:
            _warn("SLAM lost — waiting")
            _car_stop()
            time.sleep(0.5)
            continue

        cx, cz = pose
        d = _dist(cx, cz, tx, tz)

        if initial_d is None:
            initial_d = d
        in_approach = d < initial_d * 0.5     # second half of journey

        # ── Stale SLAM detection (only in real mode, in approach) ──────────
        if _mode == "real" and in_approach and last_pose is not None and pose == last_pose:
            stale_count += 1
            if stale_count >= STALE_LIMIT:
                _car_stop()
                _warn(f"STALE ×{stale_count} — waiting up to {STALE_TIMEOUT:.0f}s …")
                deadline = time.time() + STALE_TIMEOUT
                while time.time() < deadline:
                    new_pose = _get_pose(slam)
                    if new_pose is not None and new_pose != pose:
                        print("      [stale] SLAM recovered")
                        break
                    time.sleep(0.1)
                else:
                    print("      [stale] no fresh data — resuming cautiously")
                stale_count = 0
                last_pose = None
                continue
        else:
            stale_count = 0
        last_pose = pose

        # ── Print at controlled rate ──────────────────────────────────────
        now = time.time()
        if now - last_print >= PRINT_INTERVAL:
            mode_label = "SLOW" if in_approach else "FULL"
            print(f"  pos=({cx:+.3f}, {cz:+.3f})   dist={d:.3f} m   [{mode_label}]")
            last_print = now

        # ── Arrival check ─────────────────────────────────────────────────
        if d < threshold:
            _car_stop()
            _ok(f"{name}   dist={d:.3f} m   pos=({cx:+.3f}, {cz:+.3f})")
            return

        # ── Heading correction ────────────────────────────────────────────
        if heading_correction and d > 0.15:
            br  = _bearing(cx, cz, tx, tz)
            if reverse:
                br = (br + 180) % 360
            hdg = _get_heading(imu) or 0.0
            df  = _angle_diff(br, hdg)
            if abs(df) > ANGLE_THRESHOLD:
                _car_stop()
                time.sleep(0.1)
                _correct_heading(br, imu)
                stuck = 0
                continue

        # ── Stuck detection ───────────────────────────────────────────────
        if abs(last_d - d) < 0.002:
            stuck += 1
        else:
            stuck = 0
        last_d = d

        if stuck > 80:
            if _mode == "real":
                _warn("stuck — reversing")
                _car_reverse()
                time.sleep(0.4)
                _car_stop()
            else:
                print("  !! stuck — resetting")
                _car_stop()
            stuck = 0
            continue

        # ── Move ──────────────────────────────────────────────────────────
        speed = SLOW_SPEED if in_approach else FORWARD_SPEED
        if reverse:
            _car_reverse()
        else:
            _car_forward(speed)
        time.sleep(0.05 if _mode == "test" else 0.1)


# ══════════════════════════════════════════════════════════════════════════════
#  PATH PLANNING  (from path_final.py — deterministic wall-based rule)
# ══════════════════════════════════════════════════════════════════════════════

def _plan(cx: float, cz: float, station_info: dict):
    """
    Compute the L-path: elbow, standoff, and turn direction.

    Returns: (elbow_x, elbow_z, standoff_x, standoff_z, turn_left, cross, is_straight)

    Elbow rule (deterministic, from path_final.py):
      X-wall stations (+X / -X):  walk Z first, then X  →  elbow = (start_x, standoff_z)
      Z-wall stations (+Z / -Z):  walk X first, then Z  →  elbow = (standoff_x, start_z)

    Straight-line detection:
      If lateral offset < 0.03 m and target is ahead → skip elbow.

    Turn direction:
      cross = leg1_x * leg2_z − leg1_z * leg2_x
      positive cross → CCW → LEFT turn
      negative cross → CW  → RIGHT turn
    """
    sx = station_info["x"]
    sz = station_info["z"]
    orientation = station_info.get("orientation", "")

    if orientation not in ORIENTATION_VECTOR:
        raise ValueError(
            f"Unknown orientation: '{orientation}'\n"
            f"  Must be one of: {list(ORIENTATION_VECTOR.keys())}\n"
            f"  Check stations.json entry."
        )

    vx, vz = ORIENTATION_VECTOR[orientation]
    standoff_x = sx + vx * STANDOFF_DIST
    standoff_z = sz + vz * STANDOFF_DIST

    # ── Straight-line detection ───────────────────────────────────────────
    approach = _parse_approach(orientation)
    lateral = _lateral_offset(cx, cz, standoff_x, standoff_z,
                               approach[0], approach[1])
    dot = ((standoff_x - cx) * approach[0] +
           (standoff_z - cz) * approach[1])
    is_straight = lateral < STRAIGHT_THRESHOLD and dot > 0

    if is_straight:
        # No elbow needed — drive straight to standoff
        return standoff_x, standoff_z, standoff_x, standoff_z, False, 0.0, True

    # ── Elbow selection (deterministic wall-based rule) ────────────────────
    if orientation in ("-X Wall", "+X Wall"):
        elbow_x, elbow_z = cx, standoff_z        # walk Z first, then X
    else:
        elbow_x, elbow_z = standoff_x, cz        # walk X first, then Z

    leg1_x = elbow_x - cx
    leg1_z = elbow_z - cz
    leg2_x = standoff_x - elbow_x
    leg2_z = standoff_z - elbow_z
    cross  = _cross2d(leg1_x, leg1_z, leg2_x, leg2_z)

    turn_left = cross > 0

    return elbow_x, elbow_z, standoff_x, standoff_z, turn_left, cross, False


# ══════════════════════════════════════════════════════════════════════════════
#  NAVIGATOR CLASS
# ══════════════════════════════════════════════════════════════════════════════

def _align_stop():
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": False, "s": False, "d": False,
                  "total": 0, "inner": 0},
            timeout=0.5)
    except Exception:
        pass


def _spin_left(speed: int):
    """Pure left spin (a key only)."""
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": True, "s": False, "d": False,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _spin_right(speed: int):
    """Pure right spin (d key only)."""
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": False, "s": False, "d": True,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _forward(speed: int = 50):
    try:
        requests.post(CAR_URL,
            json={"w": True, "a": False, "s": False, "d": False,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _backward(speed: int = 35):
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": False, "s": True, "d": False,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _detect(detector: ColorDetector):
    """Get a single detection. Returns (found, error_px, area, frame_w)."""
    det = detector.detect_once()
    if not det["found"]:
        return False, 0, 0, det.get("frame_w", 640)
    error = det["cx"] - (det["frame_w"] // 2)
    return True, error, det["area"], det["frame_w"]


def _detect_fresh(detector: ColorDetector, flush_count: int = 3):
    """
    Flush stale buffered frames, return only the LATEST detection.
    This prevents acting on old data from before the car stopped.
    """
    result = (False, 0, 0, 640)
    for _ in range(flush_count):
        result = _detect(detector)
    return result


# ── Search: slow scan until object appears ────────────────────────────────────

def search_for_object(
    detector: ColorDetector,
    scan_direction: str = "left",
    timeout: float = SEARCH_TIMEOUT,
) -> bool:
    """Slowly rotate until the object enters the camera frame."""
    print(f"\n[search] scanning {scan_direction} (timeout {timeout}s)")
    start = time.time()

    while (time.time() - start) < timeout:
        found, error, area, _ = _detect(detector)
        if found:
            _align_stop()
            print(f"[search] ✓ found  err={error:+d}px  area={area}")
            return True

        if scan_direction == "left":
            _spin_left(SEARCH_SPIN)
        else:
            _spin_right(SEARCH_SPIN)
        time.sleep(0.05)

    _align_stop()
    print("[search] ✗ timed out")
    return False


# ── Align: iterative nudge-and-verify ────────────────────────────────────────

def align_to_object(
    detector: ColorDetector,
    dead_zone: int = DEAD_ZONE,
    confirm_frames: int = CONFIRM_FRAMES,
    timeout: float = ALIGN_TIMEOUT,
) -> bool:
    """
    Iterative nudge-and-verify alignment:
      1. Detect FRESH object position (flush stale frames first!)
      2. If off-centre: nudge in that direction
      3. Stop, settle, flush stale frames, detect again
      4. Spin speed decreases gently each iteration (coarse → fine)
      5. If stuck (motor too weak), burst at higher speed briefly
      6. Once centred for confirm_frames in a row → success
    """
    print(f"\n[align] centering  dead_zone=±{dead_zone}px  confirm={confirm_frames}")

    centred_count  = 0
    stuck_count    = 0
    last_abs_err   = 9999
    iteration      = 0
    start          = time.time()

    while (time.time() - start) < timeout:

        # 1. Get FRESH detection (flush any stale buffered frames)
        found, error, area, fw = _detect_fresh(detector)

        if not found:
            stuck_count += 1
            if stuck_count >= 20:
                print("[align] ✗ object lost")
                return False
            time.sleep(0.04)
            continue

        abs_err = abs(error)

        # 2. Check if centred
        if abs_err <= dead_zone:
            centred_count += 1
            print(f"  centred {centred_count}/{confirm_frames}  err={error:+d}px")
            if centred_count >= confirm_frames:
                _align_stop()
                print(f"[align] ✓ aligned  area={area}")
                return True
            stuck_count = 0
            time.sleep(0.05)
            continue

        # Lost centring streak
        centred_count = 0
        iteration    += 1

        # 3. Spin speed — decays gently, never below SPIN_MIN
        spin_speed = max(SPIN_MIN, SPIN_START - (iteration - 1) * SPIN_DECAY)

        # 4. Pulse duration — proportional to error size
        #    Small error → tiny pulse (0.05s), large error → bigger pulse (0.18s)
        half_frame     = fw / 2.0
        ratio          = min(abs_err / half_frame, 1.0)
        pulse_duration = 0.05 + ratio * 0.13

        # DIRECTION: object LEFT of centre (error<0) → car must turn LEFT
        #           object RIGHT of centre (error>0) → car must turn RIGHT
        direction = "L" if error < 0 else "R"
        print(f"  iter={iteration:3d}  {direction}  err={error:+d}px  "
              f"spd={spin_speed}  pulse={int(pulse_duration*1000)}ms")

        # 5. Nudge
        if error < 0:
            _spin_left(spin_speed)
        else:
            _spin_right(spin_speed)
        time.sleep(pulse_duration)
        _align_stop()

        # 6. CRITICAL: Wait for car to physically stop, then flush stale
        #    frames so next _detect_fresh reads the settled position
        time.sleep(0.12)

        # 7. Stuck detection — error hasn't changed
        if abs_err >= last_abs_err - 3:
            stuck_count += 1
            if stuck_count >= 6:
                # Wheels stuck — jog forward then backward to break friction
                print("  [align] stuck — jogging fwd/bwd to break friction")
                _forward(40)
                time.sleep(0.15)
                _align_stop()
                time.sleep(0.08)
                _backward(35)
                time.sleep(0.10)
                _align_stop()
                time.sleep(0.10)

                stuck_count = 0
                iteration   = max(0, iteration - 5)  # loosen decay
        else:
            stuck_count = 0

        last_abs_err = abs_err

    _align_stop()
    print(f"[align] ✗ timed out after {timeout:.0f}s")
    return False


# ── Approach: nudge forward → stop → verify → repeat ─────────────────────────

def approach_object(
    detector: ColorDetector,
    target_area: int = 15000,
    timeout: float = 40.0,
) -> bool:

    """
    Proportional forward approach with ZUPT-style stuck detection:
      - Drive in short bursts, verify area after each
      - Slow down as area grows (proportional speed)
      - ZUPT: if area hasn't grown over several readings, car is stuck
        → boost speed or jog backward then push harder
      - Re-align if object drifts off-centre
    """
    MAX_FWD_SPEED  = 50    # speed when far
    MIN_FWD_SPEED  = 35    # speed near target — high enough to always move!
    MAX_PULSE      = 0.20  # seconds — burst when far
    MIN_PULSE      = 0.08  # seconds — burst when close
    BOOST_SPEED    = 60    # used when stuck (short burst)
    BOOST_PULSE    = 0.15

    # ZUPT parameters
    ZUPT_WINDOW    = 5     # check area growth over last N readings
    ZUPT_MIN_GROWTH = 100  # area must grow by at least this much over window
    MAX_STUCK      = 3     # after this many stuck detections, jog backward

    print(f"\n[approach] target area >= {target_area}")
    start = time.time()
    lost  = 0
    area_history = []       # rolling window of recent area readings
    stuck_count  = 0

    while (time.time() - start) < timeout:
        # Use fresh detection to avoid stale-frame drift
        found, error, area, _ = _detect_fresh(detector)

        if not found:
            lost += 1
            if lost > 15:
                print("[approach] ✗ object lost")
                return False
            time.sleep(0.05)
            continue
        lost = 0

        # Arrived?
        if area >= target_area:
            _align_stop()
            print(f"[approach] ✓ reached target  area={area}")
            return True

        # Re-align if drifting
        if abs(error) > DEAD_ZONE:
            _align_stop()
            print(f"  [approach] drifted (err={error:+d}px) — re-aligning")
            align_to_object(detector, timeout=10.0)
            area_history.clear()
            stuck_count = 0
            continue

        # ── ZUPT: stuck detection ──
        area_history.append(area)
        if len(area_history) > ZUPT_WINDOW:
            area_history.pop(0)

        is_stuck = False
        if len(area_history) >= ZUPT_WINDOW:
            growth = area_history[-1] - area_history[0]
            if growth < ZUPT_MIN_GROWTH:
                is_stuck = True

        if is_stuck:
            stuck_count += 1
            if stuck_count >= MAX_STUCK:
                # Hard unstick: jog backward, then strong forward push
                print(f"  [ZUPT] car stuck ({stuck_count}x) — backward jog + hard push")
                _backward(40)
                time.sleep(0.18)
                _align_stop()
                time.sleep(0.10)
                _forward(BOOST_SPEED)
                time.sleep(0.25)
                _align_stop()
                time.sleep(0.10)
                stuck_count = 0
                area_history.clear()
                continue
            else:
                # Soft unstick: single boosted forward burst
                print(f"  [ZUPT] car stuck ({stuck_count}x) — boosting to {BOOST_SPEED}%")
                _forward(BOOST_SPEED)
                time.sleep(BOOST_PULSE)
                _align_stop()
                time.sleep(0.10)
                area_history.clear()
                continue

        # ── Normal proportional approach ──
        stuck_count = 0
        progress = min(area / target_area, 1.0)  # 0=far, 1=at target
        speed    = int(MAX_FWD_SPEED - progress * (MAX_FWD_SPEED - MIN_FWD_SPEED))
        pulse    = MAX_PULSE - progress * (MAX_PULSE - MIN_PULSE)

        print(f"  area={area:6d}/{target_area}  err={error:+d}px  "
              f"spd={speed}  pulse={int(pulse*1000)}ms")

        _forward(speed)
        time.sleep(pulse)
        _align_stop()
        time.sleep(0.10)  # settle before next detect

    _align_stop()
    print(f"[approach] ✗ timed out after {timeout:.0f}s")
    return False



def _run_align_sequence(color: str = "blue", target_area: int = 15000) -> bool:
    """
    Run search → align → approach after docking at a station.
    Only runs in --mode real. In test mode prints a mock message and returns True.
    """
    global _phase, _align_status
    if _mode == "test":
        print("[align] test mode — skipping align sequence")
        _align_status = "test mode — skipping"
        return True

    print("\n[align] starting align sequence")
    _align_status = "searching"
    detector = ColorDetector(stream_addr=STREAM_ADDR, target_color=color)
    _set_phase("aligning")
    try:
        found = search_for_object(detector, scan_direction="left")
        if not found:
            print("[align] object not found — skipping")
            _align_status = "object not found"
            return False

        _align_status = "aligning"
        aligned = align_to_object(detector)
        if not aligned:
            print("[align] could not align — skipping")
            _align_status = "could not align"
            return False

        _align_status = "approaching"
        close = approach_object(detector, target_area=target_area)
        if not close:
            print("[align] could not approach — skipping")
            _align_status = "could not approach"
            return False

        print("[align] ✓ aligned and in position")
        _align_status = "aligned and in position"
        return True
    except Exception as e:
        print(f"[align] error: {e}")
        _align_status = f"error: {e}"
        return False
    finally:
        _align_stop()
        detector.release()
        _set_phase("arrived")

class Navigator:
    def __init__(self, stations_file: str):
        global _stations, _mock_pose, _mock_heading

        with open(stations_file) as f:
            _stations.update(json.load(f))

        if _mode == "real":
            import zmq
            ctx = zmq.Context()
            self._imu  = _RealImuReader(ctx)
            self._slam = _RealSlamReader(ctx)
            self._wait_for_sensors()
        else:
            self._imu  = _MockImuReader()
            self._slam = _MockSlamReader()
            print("[nav] Mock mode — no real hardware")
            print(f"[nav] IMU  OK  hdg={self._imu.get():.1f}°")
            cx, cz = self._slam.get()
            print(f"[nav] SLAM OK  pos=({cx:.4f}, {cz:.4f})")

    def _wait_for_sensors(self):
        """Wait for real ZMQ sensors to start producing data."""
        _banner("WAITING FOR SENSORS")
        print(f"  IMU  → tcp://{IMU_HOST}:{IMU_PORT}")
        print(f"  SLAM → tcp://{SLAM_HOST}:{SLAM_PORT}")
        print()

        for _ in range(50):
            if self._imu.get() is not None:
                break
            time.sleep(0.1)
        if self._imu.get() is None:
            print("ERROR: no IMU data after 5 s — is the Pi reachable?")
            sys.exit(1)
        _ok(f"IMU   hdg = {self._imu.get():.1f}°")

        for _ in range(25):
            if self._slam.get() is not None:
                break
            time.sleep(0.2)
        pose = self._slam.get()
        if pose is None:
            print("ERROR: no SLAM data after 5 s — is slam_zmq running?")
            sys.exit(1)
        _ok(f"SLAM  pos = ({pose[0]:.4f}, {pose[1]:.4f})")

    # ── Departure ─────────────────────────────────────────────────────────
    def _depart(self):
        """Reverse straight back from the current station along approach axis."""
        global _docked_at

        _set_phase("departing")
        print(f"\n[nav] departing '{_docked_at}'")

        pose = _get_pose(self._slam)
        if pose is None:
            print("  [depart] WARNING: no SLAM — skipping")
            _docked_at = None
            return

        info        = _stations[_docked_at]
        orientation = info.get("orientation", "")
        approach    = _parse_approach(orientation)

        cx, cz  = pose
        clear_x = cx - approach[0] * DEPART_DIST
        clear_z = cz - approach[1] * DEPART_DIST

        print(f"  [depart] reversing from ({cx:.3f}, {cz:.3f}) "
              f"to ({clear_x:.3f}, {clear_z:.3f})  [{orientation}]")

        _drive_to("depart_clear", clear_x, clear_z,
                  WAYPOINT_THRESHOLD, self._slam, self._imu,
                  heading_correction=False, reverse=True)

        _docked_at = None
        print("  [depart] done")

    # ── Main navigation ──────────────────────────────────────────────────
    def Maps_to(self, target_name: str):
        """Navigate to a station. Blocking call."""
        global _docked_at, _path_info

        if target_name not in _stations:
            raise ValueError(f"Unknown station '{target_name}'. "
                             f"Available: {[k for k in _stations if not k.startswith('_')]}")



        station_info = _stations[target_name]
        orientation  = station_info.get("orientation", "MISSING")

        # Get current position
        pose = _get_pose(self._slam)
        if pose is None:
            print("ERROR: SLAM not available")
            return
        cx, cz = pose

        # Plan path
        elbow_x, elbow_z, standoff_x, standoff_z, turn_left, cross, is_straight = \
            _plan(cx, cz, station_info)

        direction = "LEFT" if turn_left else "RIGHT"

        # Store path info for web server
        with _state_lock:
            _path_info = {
                "start":      [cx, cz],
                "elbow":      [elbow_x, elbow_z],
                "target":     [station_info["x"], station_info["z"]],
                "standoff":   [standoff_x, standoff_z],
                "is_straight": is_straight,
            }

        if is_straight:
            _banner(f"PATH PLAN  →  {target_name}  [{orientation}]  STRAIGHT")
            print(f"  start      : ({cx:+.4f}, {cz:+.4f})")
            print(f"  standoff   : ({standoff_x:+.4f}, {standoff_z:+.4f})")
            print(f"  station    : ({station_info['x']:+.4f}, {station_info['z']:+.4f})")
            _sep()
        else:
            _banner(f"PATH PLAN  →  {target_name}  [{orientation}]")
            print(f"  start      : ({cx:+.4f}, {cz:+.4f})")
            print(f"  elbow      : ({elbow_x:+.4f}, {elbow_z:+.4f})")
            print(f"  standoff   : ({standoff_x:+.4f}, {standoff_z:+.4f})")
            print(f"  station    : ({station_info['x']:+.4f}, {station_info['z']:+.4f})")
            print()
            print(f"  spin       : 90°  {direction}  (cross={cross:+.4f})")
            print()
            print(f"  thresholds : elbow={WAYPOINT_THRESHOLD:.2f} m  "
                  f"standoff={STATION_THRESHOLD:.2f} m  "
                  f"angle={ANGLE_THRESHOLD}°  spin_tol={SPIN_TOLERANCE}°")
            _sep()
        # Depart current station if docked
        if _docked_at is not None:
            self._depart()

        t_start = time.time()

        if is_straight:
            # ── Phase 0 — spin to face standoff ───────────────────────────
            bear = _bearing(cx, cz, standoff_x, standoff_z)
            _set_phase("phase0")
            print(f"\n[Phase 0] Spinning to face target  bearing={bear:.1f}°")
            _spin_to_bearing(bear, self._imu)
            time.sleep(0.3)
            if _stop_event.is_set(): return

            # ── Phase 3 — drive straight to standoff ──────────────────────
            _set_phase("phase3")
            print(f"\n[Phase 3] Docking to {target_name}")
            _drive_to("standoff", standoff_x, standoff_z,
                      STATION_THRESHOLD, self._slam, self._imu,
                      heading_correction=True)

        else:
            # ── Phase 1 — Align & Run to Elbow ────────────────────────────
            _set_phase("phase1")
            
            # Read exact bearing to elbow coordinates
            pose = _get_pose(self._slam)
            if pose is not None:
                cx, cz = pose
            bear_to_elbow = _bearing(cx, cz, elbow_x, elbow_z)
            print(f"\n[Phase 1] Aligning to elbow bearing={bear_to_elbow:.1f}°")
            
            # Stop and perfectly align
            _car_stop()
            _spin_to_bearing(bear_to_elbow, self._imu)
            if _stop_event.is_set(): return

            # Drive straight forward to the elbow
            print(f"          Driving to elbow ({elbow_x:+.4f}, {elbow_z:+.4f})")
            _drive_to("elbow", elbow_x, elbow_z,
                      WAYPOINT_THRESHOLD, self._slam, self._imu,
                      heading_correction=True)
            if _stop_event.is_set(): return
            
            time.sleep(PHASE_PAUSE)

            # ── Phase 2 — Align at Elbow ──────────────────────────────────
            _set_phase("phase2")
            _car_stop()
            
            # Calculate bearing from elbow to target station standoff
            pose = _get_pose(self._slam)
            if pose is not None:
                cx, cz = pose
            bear_to_standoff = _bearing(cx, cz, standoff_x, standoff_z)
            print(f"\n[Phase 2] Aligning to standoff bearing={bear_to_standoff:.1f}°")
            
            # Spin perfectly to face standoff
            _spin_to_bearing(bear_to_standoff, self._imu)
            if _stop_event.is_set(): return
            
            time.sleep(PHASE_PAUSE)

            # ── Phase 3 — Final Approach & Dock ───────────────────────────
            _set_phase("phase3")
            print(f"\n[Phase 3] Docking to {target_name} standoff")
            _drive_to("standoff", standoff_x, standoff_z,
                      STATION_THRESHOLD, self._slam, self._imu,
                      heading_correction=True)

        if _stop_event.is_set():
            return

        # ── Final report ──────────────────────────────────────────────────
        _car_stop()
        elapsed = time.time() - t_start
        final   = _get_pose(self._slam)

        _set_phase("arrived")

        if final is not None:
            err = _dist(final[0], final[1], standoff_x, standoff_z)
            _banner(f"ARRIVED  ✓  {target_name}")
            print(f"  target   : ({standoff_x:+.4f}, {standoff_z:+.4f})")
            print(f"  final    : ({final[0]:+.4f}, {final[1]:+.4f})")
            print(f"  error    : {err * 100:.1f} cm")
            print(f"  time     : {elapsed:.1f} s")
            _sep()
        else:
            print(f"\n[nav] arrived at {target_name}  (no final SLAM)")

        print(f"\n[nav] arrived at {target_name}")
        _docked_at = target_name
        _run_align_sequence()


# ══════════════════════════════════════════════════════════════════════════════
#  PHASE HELPER
# ══════════════════════════════════════════════════════════════════════════════

def _set_phase(phase: str):
    global _phase
    with _state_lock:
        _phase = phase


# ══════════════════════════════════════════════════════════════════════════════
#  PUBLIC API  (called by web/server.py)
# ══════════════════════════════════════════════════════════════════════════════

_navigator = None   # Navigator instance, created by init()


def init(stations_file: str = None, mode: str = "test"):
    """Initialize the navigator. Must be called before navigate_to/stop/get_state."""
    global _navigator, _mode, _docked_at
    _mode = mode
    if stations_file is None:
        stations_file = STATIONS_FILE
    _navigator = Navigator(stations_file)
    return _navigator


def navigate_to(station_name: str) -> None:
    """Start navigation in a background thread. Non-blocking."""
    global _nav_thread, _running, _path_info
    if _navigator is None:
        raise RuntimeError("Navigator not initialised — call init() first")

    with _state_lock:
        if _running:
            print("[nav] already running — ignoring")
            return
        _running = True

    _stop_event.clear()

    def _run():
        global _running, _path_info
        try:
            _navigator.Maps_to(station_name)
        except Exception as e:
            print(f"[nav] CRASH — {e}")
            _car_stop()
        finally:
            with _state_lock:
                _running = False
                _path_info = None
            _set_phase("idle")

    _nav_thread = threading.Thread(target=_run, daemon=True)
    _nav_thread.start()


def stop() -> None:
    """Stop motors immediately. Set state to idle."""
    global _running, _path_info
    _stop_event.set()
    _car_stop()
    with _state_lock:
        _running = False
        _path_info = None
    _set_phase("idle")


def get_state() -> dict:
    """Return current state dict for /state endpoint."""
    with _mock_lock:
        pos = {"x": _mock_pose[0], "z": _mock_pose[1]}
        heading = _mock_heading

    with _state_lock:
        return {
            "pos":       pos,
            "heading":   heading,
            "phase":     _phase,
            "docked_at": _docked_at,
            "align_running": _phase == "aligning",
            "align_status": _align_status,
            "mode":      _mode,
            "running":   _running,
            "path":      _path_info,
        }


# ══════════════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════════════

def main():
    global _mode, _docked_at, _mock_pose, _mock_heading

    parser = argparse.ArgumentParser(description="L-shape navigation to a station")
    parser.add_argument("--mode", choices=["test", "real"], default="test",
                        help="test = mock sensors | real = ZMQ + Pi HTTP")
    parser.add_argument("target", nargs="?", default="station_1",
                        help="Station key from stations.json  (default: station_1)")
    parser.add_argument("--list", action="store_true",
                        help="List available stations and exit")
    parser.add_argument("--manual", action="store_true",
                        help="Press Enter between every phase (real mode only)")
    args = parser.parse_args()

    _mode = args.mode

    # ── Load stations for --list ─────────────────────────────────────────
    try:
        with open(STATIONS_FILE) as f:
            stations = json.load(f)
    except FileNotFoundError:
        print(f"ERROR: stations file not found: {STATIONS_FILE}")
        sys.exit(1)

    station_keys = [k for k in stations if not k.startswith("_")]

    if args.list:
        print(f"\nAvailable stations  (mode={_mode}):")
        for k in station_keys:
            s = stations[k]
            o = s.get("orientation", "no orientation!")
            print(f"  {k:12s}  x={s['x']:+.4f}  z={s['z']:+.4f}  [{o}]")
        print()
        return

    # ── Validate target ──────────────────────────────────────────────────
    if args.target not in stations:
        print(f"\nERROR: '{args.target}' not found in stations.json")
        print(f"  File: {STATIONS_FILE}")
        print(f"  Available keys: {station_keys}")
        print()
        print("  TIP: Key names must match exactly (underscore matters).")
        print("       Run with --list to see all stations.")
        sys.exit(1)

    # ── Init and go ──────────────────────────────────────────────────────
    nav = init(STATIONS_FILE, _mode)

    # If test mode, start at the "start" station
    if _mode == "test":
        if "start" in stations:
            with _mock_lock:
                _mock_pose = (stations["start"]["x"], stations["start"]["z"])
                _mock_heading = 180.0
            _docked_at = "start"

    if _mode == "real":
        input("\n  Press Enter to START  ▶  (camera ready?) …\n")

    nav.Maps_to(args.target)
    print("\n[nav] done")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _car_stop()
        print("\n\n  [nav] stopped by user  (Ctrl-C)")
    except Exception as e:
        _car_stop()
        print(f"\n  [nav] CRASH — {e}")
        raise
