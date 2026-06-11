import requests, time, zmq, json, sys

PI  = "http://10.213.37.191:5000/drive"
IMU = "tcp://10.213.37.191:5556"
SPIN_SPEED = 65
TIMEOUT    = 8.0
GOAL       = 90
TOLERANCE  = 6

ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.CONFLATE, 1)   # always get latest message
s.connect(IMU)
s.setsockopt_string(zmq.SUBSCRIBE, '')

def get_heading():
    try:
        return json.loads(s.recv_string())['heading_deg']
    except:
        return None

def stop():
    requests.post(PI, json={'w':False,'a':False,'s':False,'d':False,'total':0,'inner':0})

def spin(direction):
    start = get_heading()
    print(f"  start={start:.1f}  spinning {direction}")

    if direction == 'right':
        requests.post(PI, json={'w':False,'a':False,'s':False,'d':True,'total':SPIN_SPEED,'inner':SPIN_SPEED})
    else:
        requests.post(PI, json={'w':False,'a':True,'s':False,'d':False,'total':SPIN_SPEED,'inner':SPIN_SPEED})

    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        hdg = get_heading()
        if hdg is None:
            continue

        if direction == 'right':
            turned = (start - hdg) % 360
        else:
            turned = (hdg - start) % 360

        print(f"  imu={hdg:.1f}  turned={turned:.1f}")

        if turned >= GOAL - TOLERANCE:
            stop()
            time.sleep(0.3)
            final = get_heading()
            if direction == 'right':
                total = (start - final) % 360
            else:
                total = (final - start) % 360
            print(f"  DONE  turned={total:.1f} (wanted {GOAL})")
            return

        time.sleep(0.02)

    stop()
    print("  TIMEOUT — stopped")

h = get_heading()
if h is None:
    print("ERROR: no IMU")
    sys.exit(1)
print(f"IMU OK — heading={h:.1f}")

while True:
    cmd = input("\n[r=right 90  l=left 90  q=quit]: ").strip().lower()
    if cmd == 'q':
        stop()
        break
    elif cmd == 'r':
        spin('right')
    elif cmd == 'l':
        spin('left')
    else:
        print("type r or l or q")
    input("Press Enter for next move...")
