"""
spin_debug.py — Single phase spin with rapid stall-recovery.
  1. Spins in one continuous phase towards TARGET_DEG.
  2. Very rapid stall detection (.3s) -> Reverses instantly to un-stick.
  3. Post-spin SLAM recovery -> Reverses repeatedly until SLAM is locked.
Usage: python spin_debug.py
"""
import zmq, json, time, threading, requests

PI_HOST   = "10.213.37.191"
CAR_URL   = f"http://{PI_HOST}:5000/drive"
IMU_ADDR  = f"tcp://{PI_HOST}:5556"
SLAM_ADDR = "tcp://localhost:5557"

# ── tuning ──
SPIN_SPEED     = 50      
REVERSE_SPEED  = 40      
REVERSE_DUR    = 0.3     
FORWARD_SPEED  = 40
FORWARD_DUR    = 0.2     # Brief push so it doesn't go too far out of alignment

TARGET_DEG     = 86
STALL_TIMEOUT  = 0.4     # React rapidly if stuck

LOG_FILE       = "spin_slam_log.txt"

# ── shared state ──
heading = [None]
slam_pos = [None]
slam_ts = [0.0]
slam_log = []
running = True

def imu_thread(ctx):
    s = ctx.socket(zmq.SUB)
    s.setsockopt(zmq.CONFLATE, 1)
    s.setsockopt_string(zmq.SUBSCRIBE, '')
    s.connect(IMU_ADDR)
    while running:
        try:
             heading[0] = json.loads(s.recv_string())['heading_deg']
        except: pass

def slam_thread(ctx):
    s = ctx.socket(zmq.SUB)
    s.setsockopt(zmq.CONFLATE, 1)
    s.setsockopt_string(zmq.SUBSCRIBE, '')
    s.connect(SLAM_ADDR)
    s.setsockopt(zmq.RCVTIMEO, 2000)
    while running:
        try:
            data = json.loads(s.recv_string())
            x, z = data.get('x'), data.get('z')
            slam_log.append((time.time(), x, z))
            if x is not None and z is not None:
                slam_pos[0] = (x, z)
                slam_ts[0] = time.time()
        except: pass

# ── car commands ──
def stop():
    for _ in range(3):
        try:
            requests.post(CAR_URL,
                json={'w':False,'a':False,'s':False,'d':False,'total':0,'inner':0},
                timeout=1.0)
        except: pass
        time.sleep(0.05)

def spin_right(speed):
    requests.post(CAR_URL,
        json={'w':False,'a':False,'s':False,'d':True,'total':speed,'inner':speed}, timeout=1.0)

def reverse(speed, duration):
    requests.post(CAR_URL,
        json={'w':False,'a':False,'s':True,'d':False,'total':speed,'inner':int(speed*0.75)}, timeout=1.0)
    time.sleep(duration)
    stop()

def forward(speed, duration):
    requests.post(CAR_URL,
        json={'w':True,'a':False,'s':False,'d':False,'total':speed,'inner':int(speed*0.75)}, timeout=1.0)
    time.sleep(duration)
    stop()

def get_turned(start_hdg):
    hdg = heading[0]
    if hdg is None: return None, None
    return hdg, (start_hdg - hdg) % 360

# ── recovery ──
def wait_for_fresh_slam(since_time, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if slam_ts[0] > since_time and slam_pos[0] is not None:
            return True
        time.sleep(0.1)
    return False

# ── main ──
ctx = zmq.Context()
threading.Thread(target=imu_thread, args=(ctx,), daemon=True).start()
threading.Thread(target=slam_thread, args=(ctx,), daemon=True).start()

print("Waiting for IMU...")
for _ in range(40):
    if heading[0] is not None: break
    time.sleep(0.1)
if heading[0] is None:
    print("ERROR: no IMU"); exit(1)
    
print("Waiting for SLAM...")
for _ in range(20):
    if slam_pos[0] is not None: break
    time.sleep(0.2)
if slam_pos[0] is None:
    print("ERROR: no SLAM"); exit(1)

print(f"IMU OK hdg={heading[0]:.1f}° | SLAM OK pos={slam_pos[0]}")
input(f"\nPress Enter to test SINGLE-PHASE spin RIGHT {TARGET_DEG}° at {SPIN_SPEED}%...")

start_hdg = heading[0]
print(f"\n[spin] starting continuous spin...")

last_turned = 0.0
stall_time = time.time()

# ── 1. CONTINUOUS SPIN ──
while True:
    hdg, turned = get_turned(start_hdg)
    if turned is None:
        time.sleep(0.02); continue
        
    if turned >= TARGET_DEG - 2.0:
        stop()
        print(f"\n[spin] ✓ Reached target turn! ({turned:.1f}°)")
        break
        
    spin_right(SPIN_SPEED)
    time.sleep(0.02)
    
    # ── RAPID STALL DETECTION ──
    if abs(turned - last_turned) > 0.5:
        stall_time = time.time()
        last_turned = turned
    elif time.time() - stall_time > STALL_TIMEOUT:
        print(f"  ⚠ Stalled at {turned:.1f}°! Pushing forward briefly to unstick...")
        forward(FORWARD_SPEED, FORWARD_DUR)
        stall_time = time.time()
        last_turned = turned

# ── 2. SLAM RECOVERY LOOP ──
print(f"\n[recovery] Checking SLAM lock...")
check_time = time.time()
attempt = 1

while True:
    print(f"  → waiting up to 2.0s for fresh SLAM data...")
    has_slam = wait_for_fresh_slam(check_time, 2.0)
    
    if has_slam:
        print(f"  ✓ SLAM is locked at pos={slam_pos[0]}!")
        break
    else:
        print(f"  ✗ SLAM lost! Reversing briefly to show camera past features (attempt {attempt})...")
        reverse(REVERSE_SPEED, REVERSE_DUR)
        check_time = time.time()  # Reset so we look for frames hitting AFTER this reverse
        attempt += 1

# ── save log ──
running = False
time.sleep(0.5)

valid = [(t, x, z) for t, x, z in slam_log if x is not None and z is not None]
lost  = len(slam_log) - len(valid)

with open(LOG_FILE, 'w') as f:
    f.write("time_s,x,z\n")
    if valid:
        t0 = valid[0][0]
        for t, x, z in valid:
            f.write(f"{t - t0:.3f},{x:.6f},{z:.6f}\n")

print(f"\n[log] {len(valid)} valid, {lost} lost (None) out of {len(slam_log)} poses → {LOG_FILE}")
