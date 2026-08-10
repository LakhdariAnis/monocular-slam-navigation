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

# imu static drift
# TODO: All three constants below need real peek.py data from the actual car
# before they are meaningful. The simulator's noise floor is artificially zero
# and real hardware won't match these placeholders.
IMU_DRIFT_WINDOW_S = 30                # TODO: tune with real data — placeholder (increased for testing)
IMU_DRIFT_RATE_THRESHOLD = 0.00005
IMU_DRIFT_MIN_ACCEL_RATIO = 1.01

IMU_DRIFT_TOPIC = "car/anomaly/imu_static_drift"

# slam low feature
SLAM_LOW_FEATURE_WINDOW_S = 5
SLAM_LOW_FEATURE_MIN_RATE_HZ = 22

SLAM_LOW_FEATURE_TOPIC = "car/anomaly/slam_low_feature"

_last_state = {
    "tracking_loss": None,
    "motor_stall": None,
    "imu_static_drift": None,
    "slam_low_feature": None,
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


def _angle_diff(a, b):
    """Signed shortest-arc difference (b - a), result in [-180, 180)."""
    d = (b - a) % 360
    if d >= 180:
        d -= 360
    return d


def _unwrap_headings(headings):
    """Convert heading_deg values to an unwrapped series using _angle_diff.

    Removes 0/360 discontinuities so linear regression is meaningful.
    Each value represents cumulative angular change from headings[0].
    Every consecutive delta goes through _angle_diff to handle wraparound.
    """
    if not headings:
        return []
    unwrapped = [0.0]
    for i in range(1, len(headings)):
        unwrapped.append(unwrapped[-1] + _angle_diff(headings[i - 1], headings[i]))
    return unwrapped


def _least_squares_slope(ys):
    """Least-squares linear regression slope for evenly-spaced samples.

    x = sample index (0, 1, 2, ...), y = ys.
    Returns slope (change in y per sample).
    """
    n = len(ys)
    if n < 2:
        return 0.0
    sum_x = 0.0
    sum_y = 0.0
    sum_xy = 0.0
    sum_x2 = 0.0
    for i, y in enumerate(ys):
        sum_x += i
        sum_y += y
        sum_xy += i * y
        sum_x2 += i * i
    denom = n * sum_x2 - sum_x * sum_x
    if denom == 0:
        return 0.0
    return (n * sum_xy - sum_x * sum_y) / denom


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


def _get_last_known_motors(influx, before_time_s_ago):
    """Find the most recent car/motors state (w,a,s,d) at or before window start.

    Uses the same carry-forward pattern as _get_last_known_w for sparse topics.
    """
    flux = f"""
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -30m, stop: -{before_time_s_ago}s)
  |> filter(fn: (r) => r["_measurement"] == "car/motors")
  |> filter(fn: (r) => r["_field"] == "w" or r["_field"] == "a" or r["_field"] == "s" or r["_field"] == "d")
  |> sort(columns: ["_time"], desc: true)
  |> limit(n: 1)
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
"""
    query_api = influx.query_api()
    tables = query_api.query(flux, org=INFLUX_ORG)
    for table in tables:
        for record in table.records:
            return {
                "w": record.values.get("w"),
                "a": record.values.get("a"),
                "s": record.values.get("s"),
                "d": record.values.get("d"),
            }
    return None


def _run_motor_stall_crit_check(influx):
    last_w = _get_last_known_w(influx, MOTOR_STALL_WINDOW_S)
    rows = _query_flux(influx, "car/motors", ["w"], MOTOR_STALL_WINDOW_S)
    w_values = ([last_w] if last_w is not None else []) + [r.get("w") for r in rows]
    w_values = [w for w in w_values if w is not None]

    if not w_values:
        return None

    true_ratio = sum(1 for w in w_values if w is True) / len(w_values)
    if true_ratio < MOTOR_STALL_W_RATIO_THRESHOLD:
        return None

    pose_rows = _query_flux(influx, "car/slam/pose", ["x", "z"], MOTOR_STALL_WINDOW_S)
    xs = [r["x"] for r in pose_rows if r.get("x") is not None]
    zs = [r["z"] for r in pose_rows if r.get("z") is not None]

    if not xs or not zs:
        return None

    spread_m = (max(xs) - min(xs)) + (max(zs) - min(zs))
    spread_mm = spread_m * 1000
    if spread_mm >= MOTOR_STALL_SPREAD_MM_THRESHOLD:
        return None

    return {
        "type": "motor_stall", "severity": "CRIT",
        "w_true_ratio": round(true_ratio, 4),
        "position_spread_mm": round(spread_mm, 4),
        "ts": time.time(),
    }


# NOTE: motor_stall arc-drift (severity<1.0) WARN detection is intentionally
# omitted. With only a single heading source (gyro-integrated heading_deg from
# the MPU-6050), there is no way to disambiguate real arc-turning from IMU
# drift while motors are active. This is a documented limitation of single-
# sensor hardware.


def _run_imu_static_drift_check(influx):
    # --- Step 1: Stationary gate — motor commands (sparse, carry-forward) ---
    last_motors = _get_last_known_motors(influx, IMU_DRIFT_WINDOW_S)
    motor_rows = _query_flux(
        influx, "car/motors", ["w", "a", "s", "d"], IMU_DRIFT_WINDOW_S
    )
    all_motor_rows = ([last_motors] if last_motors is not None else []) + motor_rows

    if not all_motor_rows:
        all_motor_rows = [{"w": False, "a": False, "s": False, "d": False}]

    for idx, row in enumerate(all_motor_rows):
        active_motors = [k for k in ("w", "a", "s", "d") if row.get(k) is True]
        if active_motors:
            return None

    # --- Step 2: Stationary gate — IMU moving field (constant freq, no carry-forward) ---
    imu_rows = _query_flux(
        influx, "car/imu", ["heading_deg", "moving"], IMU_DRIFT_WINDOW_S
    )

    moving_rows = [r for r in imu_rows if r.get("moving") is True]
    if moving_rows:
        return None

    # --- Step 3: Sufficient data check ---
    headings = [r["heading_deg"] for r in imu_rows if r.get("heading_deg") is not None]

    if len(headings) < 4:
        return None

    # --- Step 4: Split-half linear regression trend analysis ---
    unwrapped = _unwrap_headings(headings)
    mid = len(unwrapped) // 2
    first_half = unwrapped[:mid]
    second_half = unwrapped[mid:]

    rate_first = _least_squares_slope(first_half)
    rate_second = _least_squares_slope(second_half)

    abs_rate_second = abs(rate_second)
    abs_rate_first = abs(rate_first)
    required_accel_rate = abs_rate_first * IMU_DRIFT_MIN_ACCEL_RATIO

    is_rate_significant = abs_rate_second > IMU_DRIFT_RATE_THRESHOLD
    is_accelerating = abs_rate_second > required_accel_rate

    if not (is_rate_significant and is_accelerating):
        return None
    return {
        "type": "imu_static_drift",
        "severity": "CRIT",
        "rate_first_half": round(rate_first, 6),
        "rate_second_half": round(rate_second, 6),
        "acceleration": round(abs(rate_second) - abs(rate_first), 6),
        "heading_start": headings[0],
        "heading_end": headings[-1],
        "ts": time.time(),
    }


def _run_slam_low_feature_check(influx):
    rows = _query_flux(influx, "car/slam/pose", ["ok"], SLAM_LOW_FEATURE_WINDOW_S)
    total = len(rows)

    if total == 0:
        return None

    rate_hz = total / SLAM_LOW_FEATURE_WINDOW_S

    if rate_hz >= SLAM_LOW_FEATURE_MIN_RATE_HZ:
        return None

    return {
        "type": "slam_low_feature",
        "severity": "WARN",
        "msg_count": total,
        "rate_hz": round(rate_hz, 4),
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

            result = _run_imu_static_drift_check(influx)
            current = result["severity"] if result else None
            if current != _last_state["imu_static_drift"]:
                if current is None:
                    payload_dict = {
                        "type": "imu_static_drift",
                        "severity": "CLEARED",
                        "ts": time.time(),
                    }
                else:
                    payload_dict = result
                payload = json.dumps(payload_dict)
                mqttc.publish(IMU_DRIFT_TOPIC, payload, qos=0)
                print(f"[ANOMALY] {IMU_DRIFT_TOPIC} <- {payload}")
                _write_anomaly_to_influx(influx, payload_dict)
                _last_state["imu_static_drift"] = current

            result = _run_slam_low_feature_check(influx)
            current = result["severity"] if result else None
            if current != _last_state["slam_low_feature"]:
                if current is None:
                    payload_dict = {
                        "type": "slam_low_feature",
                        "severity": "CLEARED",
                        "ts": time.time(),
                    }
                else:
                    payload_dict = result
                payload = json.dumps(payload_dict)
                mqttc.publish(SLAM_LOW_FEATURE_TOPIC, payload, qos=0)
                print(f"[ANOMALY] {SLAM_LOW_FEATURE_TOPIC} <- {payload}")
                _write_anomaly_to_influx(influx, payload_dict)
                _last_state["slam_low_feature"] = current

            state_strs = [f"{k}: {'FOUND' if v else 'NOT FOUND'}" for k, v in _last_state.items()]
            print(f"[DETECTOR] {' | '.join(state_strs)}")

            time.sleep(CHECK_INTERVAL_S)
    except KeyboardInterrupt:
        print("[DETECTOR] shutting down")
    finally:
        mqttc.loop_stop()
        mqttc.disconnect()
        influx.close()


if __name__ == "__main__":
    main()
