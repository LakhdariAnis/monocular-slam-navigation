import json
import math
import os
import time

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient

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

_last_state = {
    "tracking_loss": None,
}


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
                    payload = json.dumps(
                        {
                            "type": "tracking_loss",
                            "severity": "CLEARED",
                            "ts": time.time(),
                        }
                    )
                else:
                    payload = json.dumps(result)
                mqttc.publish(TRACKING_LOSS_TOPIC, payload, qos=0)
                print(f"[ANOMALY] {TRACKING_LOSS_TOPIC} <- {payload}")
                _last_state["tracking_loss"] = current

            print(f"[DETECTOR] tracking_loss={_last_state['tracking_loss']}")

            time.sleep(CHECK_INTERVAL_S)
    except KeyboardInterrupt:
        print("[DETECTOR] shutting down")
    finally:
        mqttc.loop_stop()
        mqttc.disconnect()
        influx.close()


if __name__ == "__main__":
    main()
