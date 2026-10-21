import sys
import time
import os
from influxdb_client import InfluxDBClient

INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = os.environ.get("INFLUXDB_TOKEN") or sys.exit("INFLUXDB_TOKEN not set: run `set -a; source monitoring/.env; set +a` first")
INFLUX_ORG = "car_org"
INFLUX_BUCKET = "car_telemetry"

def peek(measurement, fields, window_s=60):
    influx = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
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

    now = time.time()
    rows = []
    for table in tables:
        for record in table.records:
            t = record.get_time().timestamp()
            row = {"age_s": round(now - t, 2)}
            for f in fields:
                row[f] = record.values.get(f)
            rows.append(row)

    print(f"\n=== {measurement} ({fields}) — last {window_s}s — {len(rows)} rows ===")
    for r in rows:
        print(r)
    influx.close()

if __name__ == "__main__":
    # usage: python peek.py car/motors w a s d [window_seconds]
    measurement = sys.argv[1]
    rest = sys.argv[2:]
    if rest and rest[-1].isdigit():
        window_s = int(rest[-1])
        fields = rest[:-1]
    else:
        window_s = 60
        fields = rest
    peek(measurement, fields, window_s)
