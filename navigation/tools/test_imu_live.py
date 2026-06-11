import zmq, json

IMU = "tcp://10.213.37.191:5556"

ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.connect(IMU)
s.setsockopt_string(zmq.SUBSCRIBE, '')

# CONFLICT_WAIT = 0 means always return latest, drop old messages
s.setsockopt(zmq.CONFLATE, 1)

print("Reading live IMU — rotate the car and watch heading change")
print("Ctrl+C to stop\n")

while True:
    msg = s.recv_string()
    hdg = json.loads(msg)['heading_deg']
    print(f"heading={hdg:.1f}", end='\r')
