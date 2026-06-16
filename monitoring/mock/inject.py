import json
import sys

import paho.mqtt.client as mqtt

BROKER_HOST = "localhost"
BROKER_PORT = 1883
TOPIC       = "car/mock/inject"

VALID = [
    "tracking_loss",
    "motor_stall",
    "position_jump",
    "phase_timeout",
    "imu_static_drift",
    "trajectory_drift",
]


def usage():
    print(__doc__)
    sys.exit(1)


def main():
    if len(sys.argv) < 2:
        usage()

    if sys.argv[1] == "list":
        print("Available anomalies:")
        for a in VALID:
            print(f"  {a}")
        sys.exit(0)

    if len(sys.argv) != 3:
        usage()

    anomaly = sys.argv[1]
    state   = sys.argv[2].lower()

    if anomaly not in VALID:
        print(f"[ERROR] unknown anomaly '{anomaly}'")
        print(f"        valid: {', '.join(VALID)}")
        sys.exit(1)

    if state not in ("on", "off"):
        print("[ERROR] state must be 'on' or 'off'")
        sys.exit(1)

    active  = state == "on"
    payload = json.dumps({"anomaly": anomaly, "active": active})

    c = mqtt.Client(client_id="inject_cli")
    c.connect(BROKER_HOST, BROKER_PORT, keepalive=5)
    c.loop_start()
    c.publish(TOPIC, payload, qos=0)
    c.loop_stop()

    print(f"[INJECT] {anomaly} → {'ON' if active else 'OFF'}")
    print(f"[INJECT] published to {TOPIC}: {payload}")


if __name__ == "__main__":
    main()