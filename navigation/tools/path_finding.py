"""
path_finding.py — L-shape navigation, same principle as the working version.

Path for -X Wall station:
  start  (cx,   cz)          <- SLAM position
  elbow  (cx,   standoff_z)  <- same X as start, same Z as standoff
  target (standoff_x, standoff_z)  <- standoff point in front of station

The elbow is always: walk along the first axis until you're level with the
standoff, then turn and walk straight in.

Which axis is "first" depends on the station orientation:
  -X Wall / +X Wall  → standoff is left/right of station
                        → walk Z first, then X
                        → elbow = (start_x, standoff_z)

  -Z Wall / +Z Wall  → standoff is in front/behind station
                        → walk X first, then Z
                        → elbow = (standoff_x, start_z)

Turn direction is computed from the cross product of the two legs,
exactly as in the working code. Always exactly 90°.
"""

import zmq, json, time, math, requests, sys, threading
import numpy as np
from scipy.spatial import ConvexHull

# ── CONFIG ────────────────────────────────────────────────────────────────────
PI_HOST       = "10.213.37.191"
PI_PORT       = 5000
SLAM_HOST     = "localhost"
SLAM_PORT     = 5557
IMU_HOST      = "10.213.37.191"
IMU_PORT      = 5556
STATIONS_FILE = "/home/boethius/autonomous_car/navigation/stations.json"
CAR_URL       = f"http://{PI_HOST}:{PI_PORT}/drive"

# ── TUNING ────────────────────────────────────────────────────────────────────
FORWARD_SPEED      = 45
SLOW_SPEED         = 40      # approach speed (40%)
SPIN_SPEED         = 40
WAYPOINT_THRESHOLD = 0.12   # elbow arrival (relaxed to avoid overshoot)
STATION_THRESHOLD  = 0.03   # standoff arrival
ANGLE_THRESHOLD    = 15     # mid-drive correction trigger
SPIN_TOLERANCE     = 6      # spin done tolerance
STALL_TIMEOUT      = 3.0
STANDOFF_DIST      = 0.30   # metres in front of station
SLOW_DISTANCE      = 0.20   # start slowing when this close to target
STALE_LIMIT        = 5      # identical SLAM reads → stop & wait
STALE_TIMEOUT      = 3.0    # seconds to wait for fresh SLAM data

# ── ORIENTATION → standoff vector (FROM station TOWARD car approach side) ─────
ORIENTATION_VECTOR = {
    "-X Wall": (+1,  0),
    "+X Wall": (-1,  0),
    "-Z Wall": ( 0, +1),
    "+Z Wall": ( 0, -1),
}

# ── CAR COMMANDS ──────────────────────────────────────────────────────────────
def _stop():
    for _ in range(3):
        try:
            requests.post(CAR_URL,
                json={'w':False,'a':False,'s':False,'d':False,'total':0,'inner':0},
                timeout=1.0)
        except: pass
        time.sleep(0.05)

def _forward(speed=None):
    s = speed or FORWARD_SPEED
    try:
        requests.post(CAR_URL,
            json={'w':True,'a':False,'s':False,'d':False,
                  'total':s,'inner':int(s*0.75)},
            timeout=1.0)
    except Exception as e:
        print(f"  [forward] ERROR: {e}")

def _spin_left():
    try:
        requests.post(CAR_URL,
            json={'w':False,'a':True,'s':False,'d':False,
                  'total':SPIN_SPEED,'inner':SPIN_SPEED}, timeout=1.0)
    except: pass

def _spin_right():
    try:
        requests.post(CAR_URL,
            json={'w':False,'a':False,'s':False,'d':True,
                  'total':SPIN_SPEED,'inner':SPIN_SPEED}, timeout=1.0)
    except: pass

def _reverse():
    try:
        requests.post(CAR_URL,
            json={'w':False,'a':False,'s':True,'d':False,
                  'total':FORWARD_SPEED,'inner':int(FORWARD_SPEED*0.75)},
            timeout=1.0)
    except: pass

# ── IMU ───────────────────────────────────────────────────────────────────────
class ImuReader:
    def __init__(self, ctx):
        self._heading = None
        self._lock    = threading.Lock()
        self._sock = ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, '')
        self._sock.connect(f"tcp://{IMU_HOST}:{IMU_PORT}")
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                with self._lock:
                    self._heading = json.loads(
                        self._sock.recv_string())['heading_deg']
            except: pass

    def get(self):
        with self._lock:
            return self._heading

# ── SLAM ──────────────────────────────────────────────────────────────────────
class SlamReader:
    def __init__(self, ctx):
        self._pose = None
        self._lock = threading.Lock()
        self._sock = ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, '')
        self._sock.connect(f"tcp://{SLAM_HOST}:{SLAM_PORT}")
        self._sock.setsockopt(zmq.RCVTIMEO, 2000)
        time.sleep(1)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                data = json.loads(self._sock.recv_string())
                with self._lock:
                    self._pose = (data['x'], data['z'])
            except: pass

    def get(self):
        with self._lock:
            return self._pose

# ── GEOMETRY ──────────────────────────────────────────────────────────────────
def _dist(x1, z1, x2, z2):
    return math.sqrt((x2-x1)**2 + (z2-z1)**2)

def _bearing(cx, cz, tx, tz):
    return math.degrees(math.atan2(tx-cx, tz-cz))

def _angle_diff(target, current):
    return (target - current + 180) % 360 - 180

def _cross2d(ax, az, bx, bz):
    return ax * bz - az * bx

# ── MOTION PRIMITIVES ─────────────────────────────────────────────────────────
def _spin_delta(degrees, turn_left, imu):
    """Spin exactly `degrees` tracked by IMU delta."""
    start = imu.get()
    print(f"  [spin] {'left' if turn_left else 'right'} {degrees:.0f}° from {start:.1f}°")
    last_turned = 0.0
    stall_start = time.time()

    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02); continue

        turned = (hdg - start) % 360 if turn_left else (start - hdg) % 360
        print(f"  hdg={hdg:.1f}  turned={turned:.1f}")

        if turned >= degrees - SPIN_TOLERANCE:
            _stop()
            time.sleep(0.3)
            print(f"  [spin] done — turned {turned:.1f}°")
            return

        if abs(turned - last_turned) > 0.5:
            last_turned = turned
            stall_start = time.time()
        elif time.time() - stall_start > STALL_TIMEOUT:
            _stop()
            input("  STALLED — fix manually then press Enter: ")
            return

        _spin_left() if turn_left else _spin_right()
        time.sleep(0.02)


def _correct_heading(target_bear, imu):
    """Mid-drive heading correction using IMU delta."""
    print(f"  [hdg] correcting to {target_bear:.1f}°")
    stall_start = time.time()
    last_diff   = None

    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02); continue

        df = _angle_diff(target_bear, hdg)
        print(f"  hdg={hdg:.1f}  diff={df:.1f}")

        if abs(df) <= SPIN_TOLERANCE:
            _stop()
            time.sleep(0.2)
            return

        if last_diff is not None and abs(abs(df) - abs(last_diff)) < 0.1:
            if time.time() - stall_start > STALL_TIMEOUT:
                _stop()
                input("  STALLED — fix manually then press Enter: ")
                return
        else:
            stall_start = time.time()
        last_diff = df

        _spin_left() if df > 0 else _spin_right()
        time.sleep(0.02)


def _drive_to(name, tx, tz, threshold, slam, imu, heading_correction=True):
    """Drive forward until within threshold of (tx, tz)."""
    print(f"\n  → {name} ({tx:.3f}, {tz:.3f})")
    stuck  = 0
    last_d = 9999.0
    last_pose   = None
    stale_count = 0
    initial_d   = None   # set on first reading

    while True:
        pose = slam.get()
        if pose is None:
            print("  SLAM lost"); _stop(); time.sleep(0.5); continue

        # ── Stale SLAM detection (only in second half of journey) ─────
        cx_tmp, cz_tmp = pose
        d_now = _dist(cx_tmp, cz_tmp, tx, tz)
        if initial_d is None:
            initial_d = d_now
        in_approach = d_now < initial_d * 0.5

        if in_approach and last_pose is not None and pose == last_pose:
            stale_count += 1
            if stale_count >= STALE_LIMIT:
                _stop()
                cx, cz = pose
                d = _dist(cx, cz, tx, tz)
                print(f"    pos=({cx:.3f},{cz:.3f})  dist={d:.3f}m  "
                      f"[STALE×{stale_count} — stopped, waiting for fresh SLAM]")
                deadline = time.time() + STALE_TIMEOUT
                fresh = False
                while time.time() < deadline:
                    new_pose = slam.get()
                    if new_pose is not None and new_pose != pose:
                        fresh = True
                        break
                    time.sleep(0.1)
                if fresh:
                    print(f"    [STALE] fresh SLAM recovered")
                else:
                    print(f"    [STALE] no fresh data after {STALE_TIMEOUT}s — resuming cautiously")
                stale_count = 0
                last_pose = None
                continue
        else:
            stale_count = 0
        last_pose = pose

        cx, cz = pose
        d = _dist(cx, cz, tx, tz)
        print(f"    pos=({cx:.3f},{cz:.3f})  dist={d:.3f}m")

        if d < threshold:
            _stop()
            print(f"  ✓ {name}")
            return

        if heading_correction and d > 0.15:
            br  = _bearing(cx, cz, tx, tz)
            hdg = imu.get() or 0.0
            df  = _angle_diff(br, hdg)
            if abs(df) > ANGLE_THRESHOLD:
                _stop(); time.sleep(0.1)
                _correct_heading(br, imu)
                stuck = 0
                continue

        if abs(last_d - d) < 0.002:
            stuck += 1
        else:
            stuck = 0
        last_d = d

        if stuck > 80:
            print("  stuck — reversing")
            _reverse(); time.sleep(0.4); _stop(); stuck = 0
            continue

        # ── Speed control: full speed first half, slow in second half ─
        speed = SLOW_SPEED if in_approach else FORWARD_SPEED
        _forward(speed)
        time.sleep(0.1)

# ── PATH PLANNING ─────────────────────────────────────────────────────────────
def _plan(cx, cz, station_info):
    """
    Returns (elbow_x, elbow_z, standoff_x, standoff_z, turn_left).

    Elbow rule:
      X-wall stations (-X / +X): standoff is sideways → go Z first, then X
        elbow = (start_x, standoff_z)
      Z-wall stations (-Z / +Z): standoff is ahead/behind → go X first, then Z
        elbow = (standoff_x, start_z)

    This guarantees the car arrives at the standoff facing straight into
    the wall, regardless of where it started.
    """
    sx = station_info['x']
    sz = station_info['z']
    orientation = station_info.get('orientation', '')

    if orientation not in ORIENTATION_VECTOR:
        raise ValueError(f"Unknown orientation: '{orientation}'")

    vx, vz = ORIENTATION_VECTOR[orientation]
    standoff_x = sx + vx * STANDOFF_DIST
    standoff_z = sz + vz * STANDOFF_DIST

    # Which axis to travel first
    if orientation in ('-X Wall', '+X Wall'):
        # standoff is left/right of station — walk Z first, then X
        elbow_x, elbow_z = cx, standoff_z
    else:
        # standoff is in front/behind station — walk X first, then Z
        elbow_x, elbow_z = standoff_x, cz

    # Spin direction from cross product
    leg1_x, leg1_z = elbow_x - cx,           elbow_z - cz
    leg2_x, leg2_z = standoff_x - elbow_x,   standoff_z - elbow_z
    cross     = _cross2d(leg1_x, leg1_z, leg2_x, leg2_z)
    turn_left = cross < 0

    return elbow_x, elbow_z, standoff_x, standoff_z, turn_left


# ── MAIN ──────────────────────────────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('target', nargs='?', default='station1',
                        help='Station name from stations.json')
    args = parser.parse_args()

    with open(STATIONS_FILE) as f:
        stations = json.load(f)

    if args.target not in stations:
        print(f"ERROR: '{args.target}' not in stations.json")
        print(f"  Available: {list(stations.keys())}")
        sys.exit(1)

    station_info = stations[args.target]

    ctx  = zmq.Context()
    imu  = ImuReader(ctx)
    slam = SlamReader(ctx)

    print("\n[nav] waiting for IMU...")
    for _ in range(40):
        if imu.get() is not None: break
        time.sleep(0.1)
    if imu.get() is None:
        print("ERROR: no IMU"); sys.exit(1)
    print(f"[nav] IMU OK  hdg={imu.get():.1f}°")

    print("[nav] waiting for SLAM...")
    for _ in range(20):
        if slam.get() is not None: break
        time.sleep(0.2)
    pose = slam.get()
    if pose is None:
        print("ERROR: no SLAM"); sys.exit(1)
    cx, cz = pose
    print(f"[nav] SLAM OK  pos=({cx:.4f}, {cz:.4f})")

    # ── Plan ─────────────────────────────────────────────────────────────────
    elbow_x, elbow_z, standoff_x, standoff_z, turn_left = _plan(
        cx, cz, station_info)

    direction = 'left' if turn_left else 'right'
    orientation = station_info.get('orientation', '')

    print(f"\n[path] → {args.target}  [{orientation}]")
    print(f"  start    : ({cx:.3f}, {cz:.3f})")
    print(f"  elbow    : ({elbow_x:.3f}, {elbow_z:.3f})")
    print(f"  standoff : ({standoff_x:.3f}, {standoff_z:.3f})")
    print(f"  spin     : 90° {direction}")

    input("\n  Press Enter to start Phase 1 (drive to elbow)...")

    # ── Phase 1: drive to elbow ───────────────────────────────────────────────
    print(f"\n[Phase 1] drive to elbow ({elbow_x:.3f}, {elbow_z:.3f})")
    _drive_to("elbow", elbow_x, elbow_z,
              WAYPOINT_THRESHOLD, slam, imu, heading_correction=True)
    time.sleep(0.3)

    # ── Phase 2: spin 90° ─────────────────────────────────────────────────────
    input(f"  Press Enter to start Phase 2 (spin {direction} 90°)...")
    print(f"\n[Phase 2] spin {direction} 90°")
    _spin_delta(90, turn_left, imu)
    time.sleep(0.5)

    # ── Phase 3: drive straight to standoff, no heading correction ────────────
    # Spin already aligned the car. Correction here causes jitter on final approach.
    input(f"  Press Enter to start Phase 3 (dock into standoff)...")
    print(f"\n[Phase 3] drive to standoff ({standoff_x:.3f}, {standoff_z:.3f})")
    _drive_to("standoff", standoff_x, standoff_z,
              STATION_THRESHOLD, slam, imu, heading_correction=False)

    _stop()
    print(f"\n[nav] ✓ arrived at '{args.target}'")
    print(f"[nav]   final pos: {slam.get()}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _stop()
        print("\nstopped")
    except Exception as e:
        _stop()
        print(f"\n[nav] CRASH — stopped. Error: {e}")
        raise
