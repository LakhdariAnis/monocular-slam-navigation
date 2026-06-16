import json
import time

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

BROKER_HOST  = "localhost"
BROKER_PORT  = 1883

INFLUX_URL    = "http://localhost:8086"
INFLUX_TOKEN  = "super-token-123"
INFLUX_ORG    = "car_org"
INFLUX_BUCKET = "car_telemetry"

def _float_or_none(v):
    return None if v is None else float(v)


FIELD_TYPES = {
    "car/slam/pose": {
        "seq": int,
        "ok":  bool,
        "x":   _float_or_none,   # None when tracking lost → dropped
        "z":   _float_or_none,
    },
    "car/imu": {
        "heading_deg": float,
        "raw_ax":      float,
        "raw_ay":      float,
        "moving":      bool,
    },
    "car/nav/phase": {
        "phase":          str,
        "target_station": str,
    },
    "car/motors": {
        "w":     bool,
        "a":     bool,
        "s":     bool,
        "d":     bool,
        "total": int,
        "inner": int,
    },
}


def _infer_type(value):
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value)
    if isinstance(value, str):
        return str(value)
    return None   # drop


influx_client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG,
)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)


def _write_point(topic: str, payload: dict):
    ts_unix = payload.get("ts")
    if ts_unix is None:
        ts_ns = time.time_ns()
    else:
        ts_ns = int(float(ts_unix) * 1_000_000_000)

    point = Point(topic).time(ts_ns, WritePrecision.NS)

    type_map = FIELD_TYPES.get(topic)

    for key, raw_value in payload.items():
        if key == "ts":
            continue

        if type_map is not None:
            cast_fn = type_map.get(key)
            if cast_fn is None:
                continue
            typed = cast_fn(raw_value)
        else:
            typed = _infer_type(raw_value)

        if typed is None:
            continue

        point.field(key, typed)

    write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
    print(f"[WRITE] {topic:<20s}  ts={ts_unix}")


def on_connect(c, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT]  connected to {BROKER_HOST}:{BROKER_PORT}")
        c.subscribe("car/#")
        print("[MQTT]  subscribed to car/#")
    else:
        print(f"[MQTT]  connection failed rc={rc} — will retry")


def on_disconnect(c, userdata, rc):
    if rc != 0:
        print(f"[MQTT]  unexpected disconnect rc={rc} — reconnecting ...")


def on_message(c, userdata, msg):
    topic = msg.topic

    if topic == "car/mock/inject":
        return

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        print(f"[WARN]  {topic}: bad JSON — {exc}")
        return

    try:
        _write_point(topic, payload)
    except Exception as exc:
        print(f"[ERROR] {topic}: write failed — {exc}")


def main():
    print(f"[WRITER] InfluxDB → {INFLUX_URL}  bucket={INFLUX_BUCKET}")
    print(f"[WRITER] MQTT     → {BROKER_HOST}:{BROKER_PORT}")
    print()

    mqtt_client = mqtt.Client(client_id="influx_writer")
    mqtt_client.on_connect    = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message    = on_message

    while True:
        try:
            mqtt_client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            break
        except (ConnectionRefusedError, OSError) as exc:
            print(f"[MQTT]  cannot connect: {exc} — retrying in 5s")
            time.sleep(5)

    print("[WRITER] running — Ctrl-C to stop")
    mqtt_client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[WRITER] stopped by user")
    finally:
        influx_client.close()
