from __future__ import annotations

import asyncio
import collections
import json
import logging
import os
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
import uvicorn
from fastapi import FastAPI, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from influxdb_client import InfluxDBClient

MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883

INFLUX_URL    = "http://localhost:8086"
INFLUX_TOKEN  = "super-token-123"
INFLUX_ORG    = "car_org"
INFLUX_BUCKET = "car_telemetry"

BACKEND_PORT = int(os.environ.get("BACKEND_PORT", "8000"))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("backend")

latest: dict[str, Any] = {}
ws_clients: set[WebSocket] = set()
ws_clients_lock = asyncio.Lock()

slam_timestamps: collections.deque = collections.deque(maxlen=60)
anomaly_feed: collections.deque = collections.deque(maxlen=50)

# same ZMQ pattern as the SLAM project
_mqtt_queue: asyncio.Queue | None = None
_loop: asyncio.AbstractEventLoop | None = None

_mqtt_client: mqtt.Client | None = None

_motor_stall_severity: str = "ok"
_motor_stall_freeze_streak: int = 0
_motor_stall_arc_side: str | None = None
_sim_speed: float = 1.0
_position_jump_probability: float = 0.0
_position_jump_last: float | None = None
_imu_drift_elapsed_s: float = 0.0
_imu_drift_accumulated_deg: float = 0.0


def _on_connect(client: mqtt.Client, userdata, flags, rc):
    if rc == 0:
        log.info("MQTT connected to %s:%s", MQTT_BROKER_HOST, MQTT_BROKER_PORT)
        client.subscribe("car/#")
        log.info("MQTT subscribed to car/#")
    else:
        log.warning("MQTT connection failed rc=%s — will retry", rc)


def _on_disconnect(client: mqtt.Client, userdata, rc):
    if rc != 0:
        log.warning("MQTT unexpected disconnect rc=%s — reconnecting …", rc)


def _on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
    """Called in the paho network thread — must be non-blocking."""
    global _motor_stall_severity, _motor_stall_freeze_streak, _motor_stall_arc_side, _sim_speed, _position_jump_probability, _position_jump_last, _imu_drift_elapsed_s, _imu_drift_accumulated_deg
    topic = msg.topic

    if topic == "car/mock/inject":
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if payload.get("anomaly") == "motor_stall" and not payload.get("active"):
            _motor_stall_severity = "ok"
            _motor_stall_freeze_streak = 0
        return

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        log.warning("MQTT bad JSON on %s: %s", topic, exc)
        return

    if topic == "car/anomaly/motor_stall":
        sev = payload.get("severity", "WARN")
        _motor_stall_severity = sev.lower()
        _motor_stall_freeze_streak = payload.get("freeze_streak", 0)

    elif topic == "car/mock/motor_stall_arc":
        raw = payload.get("arc_side")
        if raw == 1:
            _motor_stall_arc_side = "RIGHT"
        elif raw == -1:
            _motor_stall_arc_side = "LEFT"
        else:
            _motor_stall_arc_side = None

    elif topic == "car/anomaly/position_jump":
        _position_jump_last = payload.get("ts", time.time())

    elif topic == "car/imu":
        _imu_drift_elapsed_s = float(payload.get("drift_elapsed_s", 0.0))
        _imu_drift_accumulated_deg = float(payload.get("accumulated_drift_deg", 0.0))

    log.info("MQTT ← %s  %s", topic, json.dumps(payload))

    if _mqtt_queue is not None and _loop is not None:
        _loop.call_soon_threadsafe(_mqtt_queue.put_nowait, (topic, payload))


def _start_mqtt_thread():
    global _mqtt_client
    _mqtt_client = mqtt.Client(client_id="dashboard_backend")
    _mqtt_client.on_connect    = _on_connect
    _mqtt_client.on_disconnect = _on_disconnect
    _mqtt_client.on_message    = _on_message

    def _connect_loop():
        while True:
            try:
                _mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=60)
                break
            except (ConnectionRefusedError, OSError) as exc:
                log.warning("MQTT cannot connect: %s — retrying in 5s", exc)
                time.sleep(5)
        _mqtt_client.loop_forever()

    t = threading.Thread(target=_connect_loop, daemon=True)
    t.start()
    return _mqtt_client


async def _mqtt_consumer():
    assert _mqtt_queue is not None
    while True:
        topic, payload = await _mqtt_queue.get()

        latest[topic] = payload

        if topic == "car/slam/pose":
            slam_timestamps.append(time.time())

        snapshot = json.dumps(latest)
        async with ws_clients_lock:
            stale: list[WebSocket] = []
            for ws in ws_clients:
                try:
                    await ws.send_text(snapshot)
                except Exception:
                    stale.append(ws)
            for ws in stale:
                ws_clients.discard(ws)

        if topic in ("car/anomaly/tracking_loss", "car/anomaly/motor_stall"):
            if topic == "car/anomaly/motor_stall" and "motor_active_ratio" not in payload:
                continue
            keys = payload.keys()
            entry = {k: payload.get(k) for k in keys}
            anomaly_feed.append(entry)
            msg = json.dumps({"_anomaly": True, "entry": entry})
            async with ws_clients_lock:
                stale = []
                for ws in ws_clients:
                    try:
                        await ws.send_text(msg)
                    except Exception:
                        stale.append(ws)
                for ws in stale:
                    ws_clients.discard(ws)


_influx_client: InfluxDBClient | None = None


def _query_influx(measurement: str, fields: list[str], minutes: int) -> list[dict]:
    assert _influx_client is not None
    query_api = _influx_client.query_api()

    field_filters = " or ".join(f'r["_field"] == "{f}"' for f in fields)

    flux = f"""
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -{minutes}m)
  |> filter(fn: (r) => r["_measurement"] == "{measurement}")
  |> filter(fn: (r) => {field_filters})
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
  |> sort(columns: ["_time"])
"""
    tables = query_api.query(flux, org=INFLUX_ORG)

    results: list[dict] = []
    for table in tables:
        for record in table.records:
            row: dict[str, Any] = {"ts": record.get_time().isoformat()}
            for f in fields:
                row[f] = record.values.get(f)
            results.append(row)
    return results


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _mqtt_queue, _loop, _influx_client

    # took a while to get the shutdown ordering right here
    _loop = asyncio.get_running_loop()
    _mqtt_queue = asyncio.Queue()

    _influx_client = InfluxDBClient(
        url=INFLUX_URL,
        token=INFLUX_TOKEN,
        org=INFLUX_ORG,
    )
    log.info("InfluxDB client → %s  bucket=%s", INFLUX_URL, INFLUX_BUCKET)

    _start_mqtt_thread()
    consumer_task = asyncio.create_task(_mqtt_consumer())

    yield

    consumer_task.cancel()
    if _influx_client:
        _influx_client.close()
    log.info("Backend shutdown complete")


app = FastAPI(
    title="Car Dashboard Backend",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ─────────────────────────────────────────────
# WEBSOCKET  /ws
# ─────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    log.info("WebSocket client connected (%s)", ws.client)

    async with ws_clients_lock:
        ws_clients.add(ws)

    try:
        if latest:
            await ws.send_text(json.dumps(latest))
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        log.info("WebSocket client disconnected (%s)", ws.client)
    except Exception as exc:
        log.warning("WebSocket error: %s", exc)
    finally:
        async with ws_clients_lock:
            ws_clients.discard(ws)


@app.get("/history/slam")
async def history_slam(minutes: int = Query(default=10, ge=1, le=1440)):
    rows = _query_influx(
        measurement="car/slam/pose",
        fields=["x", "z", "ok"],
        minutes=minutes,
    )
    return rows


@app.get("/history/imu")
async def history_imu(minutes: int = Query(default=10, ge=1, le=1440)):
    rows = _query_influx(
        measurement="car/imu",
        fields=["heading_deg", "raw_ax", "raw_ay", "moving"],
        minutes=minutes,
    )
    return rows


@app.get("/history/motors")
async def history_motors(minutes: int = Query(default=10, ge=1, le=1440)):
    rows = _query_influx(
        measurement="car/motors",
        fields=["total", "inner", "w", "a", "s", "d"],
        minutes=minutes,
    )
    return rows


@app.get("/history/phase")
async def history_phase(minutes: int = Query(default=10, ge=1, le=1440)):
    rows = _query_influx(
        measurement="car/nav/phase",
        fields=["phase", "target_station"],
        minutes=minutes,
    )
    return rows


class InjectRequest(BaseModel):
    anomaly: str
    active: bool


class PublishRequest(BaseModel):
    topic: str
    payload: dict


@app.post("/api/publish")
async def publish_mqtt(req: PublishRequest):
    if _mqtt_client is not None:
        _mqtt_client.publish(req.topic, json.dumps(req.payload), qos=0)
        log.info("PUBLISH → %s  %s", req.topic, req.payload)
        return {"ok": True}
    else:
        log.warning("PUBLISH failed — MQTT client not connected")
        return {"ok": False, "error": "MQTT not connected"}


@app.post("/api/inject")
async def inject_anomaly(req: InjectRequest):
    payload = json.dumps({"anomaly": req.anomaly, "active": req.active})
    if _mqtt_client is not None:
        _mqtt_client.publish("car/mock/inject", payload, qos=0)
        log.info("INJECT → %s = %s", req.anomaly, req.active)
        return {"ok": True, "anomaly": req.anomaly, "active": req.active}
    else:
        log.warning("INJECT failed — MQTT client not connected")
        return {"ok": False, "error": "MQTT not connected"}


@app.post("/api/goto")
async def goto(request: Request):
    body = await request.json()
    target = body.get("target", "")
    if not target:
        return {"status": "error", "reason": "missing target"}
    _mqtt_client.publish("car/mock/goto", json.dumps({"target": target}))
    return {"status": "sent", "target": target}


@app.post("/sim/reset")
async def sim_reset():
    _mqtt_client.publish("car/mock/reset", json.dumps({"station": "start"}))
    return {"status": "reset sent"}


@app.get("/anomalies")
async def get_anomalies():
    return list(anomaly_feed)


@app.post("/anomalies/clear")
async def clear_anomalies():
    anomaly_feed.clear()
    return {"status": "cleared"}


@app.get("/live/slam_rate")
async def live_slam_rate():
    now = time.time()
    cutoff = now - 2.0
    count = sum(1 for t in slam_timestamps if t > cutoff)
    msgs_per_sec = count / 2.0  # thresholds tuned on bench with the real SLAM rig
    if msgs_per_sec >= 20:
        status = "ok"
    elif msgs_per_sec >= 10:
        status = "warn"
    else:
        status = "critical"
    return {"msgs_per_sec": msgs_per_sec, "status": status}


@app.get("/live/motor_stall")
async def live_motor_stall():
    return {
        "severity": _motor_stall_severity,
        "freeze_streak": _motor_stall_freeze_streak,
        "arc_side": _motor_stall_arc_side,
    }


@app.get("/live/sim_speed")
async def live_sim_speed():
    return {"sim_speed": _sim_speed}


@app.post("/control/sim_speed")
async def set_sim_speed(payload: dict):
    global _sim_speed
    speed = max(0.25, min(4.0, float(payload["speed"])))
    _sim_speed = speed
    if _mqtt_client is not None:
        _mqtt_client.publish("car/mock/sim_speed", json.dumps({"speed": speed}))
    return {"ok": True, "speed": speed}


@app.get("/live/position_jump")
async def live_position_jump():
    return {
        "probability": _position_jump_probability,
        "last_jump": _position_jump_last,
    }


@app.post("/control/position_jump_params")
async def set_position_jump_params(payload: dict):
    global _position_jump_probability
    prob = max(0.0, min(1.0, float(payload.get("probability", 0.0))))
    _position_jump_probability = prob
    if _mqtt_client is not None:
        _mqtt_client.publish(
            "car/mock/position_jump_params",
            json.dumps({"probability": prob}),
        )
    return {"ok": True, "probability": prob}


@app.post("/control/imu_drift_reset")
async def reset_imu_drift():
    if _mqtt_client is not None:
        _mqtt_client.publish("car/mock/imu_drift_reset", json.dumps({}))
    return {"ok": True}


@app.get("/live")
async def live():
    return {
        "imu_drift": {
            "elapsed_s": _imu_drift_elapsed_s,
            "accumulated_deg": _imu_drift_accumulated_deg,
        }
    }


_FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"
if _FRONTEND_DIR.is_dir():
    app.mount("/", StaticFiles(directory=str(_FRONTEND_DIR), html=True), name="frontend")


if __name__ == "__main__":
    log.info("Starting dashboard backend on port %s", BACKEND_PORT)
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=BACKEND_PORT,
        log_level="info",
    )
