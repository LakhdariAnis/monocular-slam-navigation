from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
import uvicorn
from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
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

# asyncio.Queue bridging the paho-mqtt thread → async consumer
# Created lazily inside the running event loop (see lifespan)
_mqtt_queue: asyncio.Queue | None = None
_loop: asyncio.AbstractEventLoop | None = None

# Reference to the paho MQTT client (for publishing inject commands)
_mqtt_client: mqtt.Client | None = None

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
    topic = msg.topic

    # Ignore the inject control topic
    if topic == "car/mock/inject":
        return

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        log.warning("MQTT bad JSON on %s: %s", topic, exc)
        return

    log.info("MQTT ← %s  %s", topic, json.dumps(payload))

    # Thread-safe enqueue into the asyncio world
    if _mqtt_queue is not None and _loop is not None:
        _loop.call_soon_threadsafe(_mqtt_queue.put_nowait, (topic, payload))


def _start_mqtt_thread():
    global _mqtt_client
    _mqtt_client = mqtt.Client(client_id="dashboard_backend")
    _mqtt_client.on_connect    = _on_connect
    _mqtt_client.on_disconnect = _on_disconnect
    _mqtt_client.on_message    = _on_message

    def _connect_loop():
        import time
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

# CORS — allow all origins (frontend served on a different port)
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


@app.post("/inject")
async def inject_anomaly(req: InjectRequest):
    payload = json.dumps({"anomaly": req.anomaly, "active": req.active})
    if _mqtt_client is not None:
        _mqtt_client.publish("car/mock/inject", payload, qos=0)
        log.info("INJECT → %s = %s", req.anomaly, req.active)
        return {"ok": True, "anomaly": req.anomaly, "active": req.active}
    else:
        log.warning("INJECT failed — MQTT client not connected")
        return {"ok": False, "error": "MQTT not connected"}


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
