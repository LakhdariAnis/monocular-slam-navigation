import json
import math
import os
import time

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883

INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = os.environ.get("INFLUXDB_TOKEN", "super-token-123")
INFLUX_ORG = "car_org"
INFLUX_BUCKET = "car_telemetry"

CHECK_INTERVAL_S = 2

# tracking loss
TRACKING_LOSS_WINDOW_S = 5
TRACKING_LOSS_THRESHOLD_RATIO = 0.6

TRACKING_LOSS_TOPIC = "car/anomaly/tracking_loss"

# motor stall
MOTOR_STALL_WINDOW_S = 6
MOTOR_STALL_W_RATIO_THRESHOLD = 0.7
MOTOR_STALL_SPREAD_MM_THRESHOLD = 4

MOTOR_STALL_TOPIC = "car/anomaly/motor_stall"

_last_state = {
    "tracking_loss": None,
    "motor_stall": None,
}


def _write_anomaly_to_influx(influx, payload_dict):
    write_api = influx.write_api(write_options=SYNCHRONOUS)
    ts_unix = payload_dict.get("ts", time.time())
    ts_ns = int(float(ts_unix) * 1_000_000_000)
    
    point = Point("car/anomaly") \
        .tag("type", payload_dict.get("type")) \
        .tag("severity", payload_dict.get("severity")) \
        .time(ts_ns, WritePrecision.NS)
        
    for k, v in payload_dict.items():
        if k not in ("type", "severity"):
            point.field(k, v)
            
    write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)



def _query_flux(influx, measurement, fields, window_s):
    field_filters = " or ".join(f'r["_field"] == "{f}"' for f in fields)
    flux = f"""
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -{window_s}s)
  |> filter(fn: (r) => r["_measurement"] == "{measurement}")
  |> filter(fn: (r) => {field_filters})
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
  |> sort(columns: ["_time"])
"""
    query_api = influx.query_api()
    tables = query_api.query(flux, org=INFLUX_ORG)
    results = []
    for table in tables:
        for record in table.records:
            row = {}
            for f in fields:
                row[f] = record.values.get(f)
            results.append(row)
    return results


def _query_flux_recent(influx, measurement, fields, window_s, limit_n):
    field_filters = " or ".join(f'r["_field"] == "{f}"' for f in fields)
    flux = f"""
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -{window_s}s)
  |> filter(fn: (r) => r["_measurement"] == "{measurement}")
  |> filter(fn: (r) => {field_filters})
  |> sort(columns: ["_time"], desc: true)
  |> limit(n: {limit_n})
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
"""
    query_api = influx.query_api()
    tables = query_api.query(flux, org=INFLUX_ORG)
    results = []
    for table in tables:
        for record in table.records:
            row = {}
            for f in fields:
                row[f] = record.values.get(f)
            results.append(row)
    results.reverse()
    return results


def _run_tracking_loss_check(influx):
    rows = _query_flux(influx, "car/slam/pose", ["ok"], TRACKING_LOSS_WINDOW_S)
    total = len(rows)
    false_count = sum(1 for r in rows if r.get("ok") is False)

    if total == 0:
        return None

    false_ratio = false_count / total

    if false_ratio <= TRACKING_LOSS_THRESHOLD_RATIO:
        return None

    return {
        "type": "tracking_loss",
        "severity": "CRIT",
        "false_count": false_count,
        "total_count": total,
        "false_ratio": round(false_ratio, 4),
        "ts": time.time(),
    }


def _get_last_known_w(influx, before_time_s_ago):
    """Find the most recent car/motors 'w' value at or before window start."""
    flux = f"""
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -30m, stop: -{before_time_s_ago}s)
  |> filter(fn: (r) => r["_measurement"] == "car/motors")
  |> filter(fn: (r) => r["_field"] == "w")
  |> sort(columns: ["_time"], desc: true)
  |> limit(n: 1)
"""
    query_api = influx.query_api()
    tables = query_api.query(flux, org=INFLUX_ORG)
    for table in tables:
        for record in table.records:
            return record.get_value()
    return None  # no prior state found


def _run_motor_stall_crit_check(influx):
    last_w = _get_last_known_w(influx, MOTOR_STALL_WINDOW_S)
    rows = _query_flux(influx, "car/motors", ["w"], MOTOR_STALL_WINDOW_S)
    w_values = ([last_w] if last_w is not None else []) + [r.get("w") for r in rows]
    w_values = [w for w in w_values if w is not None]

    if not w_values:
        print("[DEBUG motor_stall] no w_values -> None")
        return None

    true_ratio = sum(1 for w in w_values if w is True) / len(w_values)
    if true_ratio < MOTOR_STALL_W_RATIO_THRESHOLD:
        print(f"[DEBUG motor_stall] last_w={last_w} rows={rows} ratio={true_ratio} -> below threshold, None")
        return None

    pose_rows = _query_flux(influx, "car/slam/pose", ["x", "z"], MOTOR_STALL_WINDOW_S)
    xs = [r["x"] for r in pose_rows if r.get("x") is not None]
    zs = [r["z"] for r in pose_rows if r.get("z") is not None]

    if not xs or not zs:
        print("[DEBUG motor_stall] no pose data -> None")
        return None

    spread_m = (max(xs) - min(xs)) + (max(zs) - min(zs))
    spread_mm = spread_m * 1000
    if spread_mm >= MOTOR_STALL_SPREAD_MM_THRESHOLD:
        print(f"[DEBUG motor_stall] spread_mm={spread_mm} -> too large, None")
        return None

    return {
        "type": "motor_stall", "severity": "CRIT",
        "w_true_ratio": round(true_ratio, 4),
        "position_spread_mm": round(spread_mm, 4),
        "ts": time.time(),
    }


def main():
    influx = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)

    mqttc = mqtt.Client(client_id="anomaly_detector")
    mqttc.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=60)
    mqttc.loop_start()

    print("[DETECTOR] anomaly detector started — every 2s")

    try:
        while True:
            result = _run_tracking_loss_check(influx)
            current = result["severity"] if result else None
            if current != _last_state["tracking_loss"]:
                if current is None:
                    payload_dict = {
                        "type": "tracking_loss",
                        "severity": "CLEARED",
                        "ts": time.time(),
                    }
                else:
                    payload_dict = result
                payload = json.dumps(payload_dict)
                mqttc.publish(TRACKING_LOSS_TOPIC, payload, qos=0)
                print(f"[ANOMALY] {TRACKING_LOSS_TOPIC} <- {payload}")
                _write_anomaly_to_influx(influx, payload_dict)
                _last_state["tracking_loss"] = current

            result = _run_motor_stall_crit_check(influx)
            current = result["severity"] if result else None
            if current != _last_state["motor_stall"]:
                if current is None:
                    payload_dict = {
                        "type": "motor_stall",
                        "severity": "CLEARED",
                        "ts": time.time(),
                    }
                else:
                    payload_dict = result
                payload = json.dumps(payload_dict)
                mqttc.publish(MOTOR_STALL_TOPIC, payload, qos=0)
                print(f"[ANOMALY] {MOTOR_STALL_TOPIC} <- {payload}")
                _write_anomaly_to_influx(influx, payload_dict)
                _last_state["motor_stall"] = current

            print(f"[DETECTOR] tracking_loss={_last_state['tracking_loss']}")
            print(f"[DETECTOR] motor_stall={_last_state['motor_stall']}")

            time.sleep(CHECK_INTERVAL_S)
    except KeyboardInterrupt:
        print("[DETECTOR] shutting down")
    finally:
        mqttc.loop_stop()
        mqttc.disconnect()
        influx.close()


if __name__ == "__main__":
    main()
