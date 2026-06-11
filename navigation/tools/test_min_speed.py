"""
test_min_speed.py — Find the absolute lowest speed that spins the car.
Increments speed by 5% each time you press Enter.
Spins for 0.5 seconds, then stops and reports the rotation speed.
"""
import zmq, json, time, threading, requests

PI_HOST   = "10.213.37.191"
CAR_URL   = f"http://{PI_HOST}:5000/drive"
IMU_ADDR  = f"tcp://{PI_HOST}:5556"

heading = [None]
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

def spin_right(speed):
    requests.post(CAR_URL,
        json={'w':False,'a':False,'s':False,'d':True,
              'total':speed,'inner':speed}, timeout=1.0)

def stop():
    for _ in range(3):
        try:
            requests.post(CAR_URL,
                json={'w':False,'a':False,'s':False,'d':False,'total':0,'inner':0},
                timeout=1.0)
        except: pass
        time.sleep(0.05)

def main():
    ctx = zmq.Context()
    threading.Thread(target=imu_thread, args=(ctx,), daemon=True).start()
    
    print("Waiting for IMU...")
    for _ in range(40):
        if heading[0] is not None: break
        time.sleep(0.1)
    if heading[0] is None:
        print("ERROR: no IMU"); exit(1)

    print("\n--- Minimum Spin Speed Tester ---")
    print("This will spin the car right for 0.5 seconds and stop.")
    
    speed = 20
    try:
        while True:
            input(f"\nPress Enter to test RIGHT spin at {speed}%...")
            
            start_hdg = heading[0]
            spin_right(speed)
            
            # Spin for 0.5 seconds
            time.sleep(0.5)
            stop()
            time.sleep(0.5) # Wait for coasting to stop
            
            end_hdg = heading[0]
            if start_hdg is not None and end_hdg is not None:
                turned = (start_hdg - end_hdg) % 360
                
                # If turned is huge (e.g. 359), it probably jittered backwards. Clamp it.
                if turned > 180:
                    turned = 0.0
                
                print(f"  Result: At {speed}%, car turned {turned:.1f}°")
                if turned < 2.0:
                    print("  -> Car did NOT move (stalled)")
                else:
                    print(f"  -> Car MOVED! Rotation rate: {turned / 0.5:.1f}°/s")
            
            speed += 5
            if speed > 100:
                break
    except KeyboardInterrupt:
        stop()
        print("\nTest aborted.")

if __name__ == "__main__":
    main()
