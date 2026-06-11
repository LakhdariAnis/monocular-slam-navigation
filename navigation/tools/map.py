#!/usr/bin/env python3
"""
live_map.py — Live car position on map using matplotlib.
Reads from ZMQ port 5557, applies calibration transform, shows on map_final.png.

Usage:
    cd ~/autonomous_car
    python3 test/live_map.py
"""

import sys
import json
import threading
from pathlib import Path

import numpy as np
import cv2
import zmq
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Paths ──────────────────────────────────────────────────────────────────────
ROOT        = Path(__file__).resolve().parent.parent
MAP_IMG     = ROOT / "data" / "map_final.png"
CAL_FILE    = ROOT / "webui" / "data" / "calibration.json"
ZMQ_PORT    = 5557
ROOM_W      = 400.0
ROOM_H      = 800.0

# ── Load calibration ───────────────────────────────────────────────────────────
if not CAL_FILE.exists():
    print(f"[ERROR] calibration.json not found at {CAL_FILE}")
    sys.exit(1)

with open(CAL_FILE) as f:
    cal = json.load(f)

H = np.float32(cal["transform_matrix"])

def slam_to_cm(x, z):
    pt = np.float32([[[x, z]]])
    result = cv2.perspectiveTransform(pt, H)
    return float(result[0][0][0]), float(result[0][0][1])

# ── Load map image ─────────────────────────────────────────────────────────────
if not MAP_IMG.exists():
    print(f"[ERROR] map_final.png not found at {MAP_IMG}")
    sys.exit(1)

img = cv2.imread(str(MAP_IMG))
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# ── Shared state ───────────────────────────────────────────────────────────────
state = {
    "x": ROOM_W / 2,
    "y": ROOM_H / 2,
    "heading": 0.0,
    "ok": False,
    "trail": [],
}
lock = threading.Lock()

# ── ZMQ thread ─────────────────────────────────────────────────────────────────
def zmq_thread():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://localhost:{ZMQ_PORT}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.setsockopt(zmq.RCVTIMEO, 500)

    print(f"[ZMQ] Subscribed to port {ZMQ_PORT}")

    while True:
        try:
            msg = sock.recv_string()
            data = json.loads(msg)
            if not data.get("ok", False):
                continue

            rx, ry = slam_to_cm(data["x"], data["z"])
            heading = data.get("heading", 0.0)

            with lock:
                state["x"] = rx
                state["y"] = ry
                state["heading"] = heading
                state["ok"] = True
                state["trail"].append((rx, ry))
                if len(state["trail"]) > 300:
                    state["trail"].pop(0)

        except zmq.Again:
            with lock:
                state["ok"] = False
        except Exception as e:
            print(f"[ZMQ] Error: {e}")

t = threading.Thread(target=zmq_thread, daemon=True)
t.start()

# ── Plot ───────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(6, 10))
plt.tight_layout(pad=1.0)

# Map image: x=0..400 left→right, y=0..800 top→bottom
ax.imshow(img, extent=[0, ROOM_W, ROOM_H, 0], aspect='auto')
ax.set_xlim(0, ROOM_W)
ax.set_ylim(ROOM_H, 0)
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.set_title("Live Car Position")

# Marker positions
markers_cm = {0: (344.9, 0), 4: (138.8, 800), 6: (400, 492.9), 7: (0, 355.2)}
for mid, (mx, my) in markers_cm.items():
    ax.plot(mx, my, 's', color='yellow', markersize=10, zorder=5)
    ax.text(mx + 8, my + 12, f"M{mid}", color='yellow', fontsize=8, fontweight='bold')

trail_line, = ax.plot([], [], '-', color='cyan', alpha=0.5, linewidth=1, zorder=6)
car_dot,    = ax.plot([], [], 'o', color='red', markersize=12, zorder=7)
status_txt  = ax.text(10, 30, "Waiting for SLAM...", color='white',
                      fontsize=9, bbox=dict(boxstyle='round', facecolor='black', alpha=0.6))

def update(_):
    with lock:
        x = state["x"]
        y = state["y"]
        heading = state["heading"]
        ok = state["ok"]
        trail = list(state["trail"])

    car_dot.set_data([x], [y])
    car_dot.set_color('red' if ok else 'gray')

    if trail:
        tx, ty = zip(*trail)
        trail_line.set_data(tx, ty)

    status = f"({'OK' if ok else 'LOST'})  x={x:.1f}cm  y={y:.1f}cm  h={heading:.1f}°"
    status_txt.set_text(status)
    status_txt.get_bbox_patch().set_facecolor('green' if ok else 'red')

    return car_dot, trail_line, status_txt

from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)

print("[MAP] Live map running. Close window to exit.")
plt.show()
