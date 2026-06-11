#!/usr/bin/env python3
"""
anchor_test.py — Test startup anchor fix.

Subscribes to slam_zmq output (port 5557), waits for SLAM to stabilize,
computes offset from M4's known SLAM world position, applies it to all poses,
prints corrected coordinates so you can verify on the map.

Run this WHILE slam_zmq.py is running.
Usage:
    python3 anchor_test.py
"""

import json
import time
import numpy as np
import cv2
import zmq
from pathlib import Path

# ── Config ─────────────────────────────────────────────────────────────────
ZMQ_PORT     = 5557
WARMUP_FRAMES = 30        # frames to average for stable startup position
ANCHOR_MARKER = 4         # which marker you're standing near at startup

# Known SLAM world positions from MarkerPositions.txt
MARKER_SLAM_POS = {
    0: ( 4.0489,  2.5146),   # (tx, tz)
    4: (-2.7503, -1.4754),
    6: ( 0.8423, -2.2559),
    7: (-0.1068,  3.2012),
}

# Calibration transform
CAL_FILE = Path.home() / "autonomous_car/webui/data/calibration.json"

# Room bounds
ROOM_W, ROOM_H = 400, 800

# ── Load calibration ────────────────────────────────────────────────────────
cal = json.loads(CAL_FILE.read_text())
H = np.float32(cal["transform_matrix"])

def slam_to_cm(x, z):
    pt = np.float32([[[x, z]]])
    r  = cv2.perspectiveTransform(pt, H)
    return float(r[0][0][0]), float(r[0][0][1])

# ── Connect ─────────────────────────────────────────────────────────────────
ctx  = zmq.Context()
sub  = ctx.socket(zmq.SUB)
sub.connect(f"tcp://localhost:{ZMQ_PORT}")
sub.setsockopt(zmq.SUBSCRIBE, b"")
sub.setsockopt(zmq.RCVTIMEO, 3000)

print(f"\n{'='*60}")
print(f"  Startup Anchor Test — standing near M{ANCHOR_MARKER}")
print(f"{'='*60}")
print(f"  Collecting {WARMUP_FRAMES} stable frames ...")
print()

# ── Phase 1: collect startup positions ─────────────────────────────────────
samples = []
total   = 0

while len(samples) < WARMUP_FRAMES:
    try:
        msg = json.loads(sub.recv_string())
    except zmq.Again:
        print("  [TIMEOUT] No SLAM data. Is slam_zmq.py running?")
        continue

    total += 1
    if not msg.get("ok"):
        print(f"  [{total:3d}] LOST")
        continue

    x, z = msg["x"], msg["z"]
    samples.append((x, z))
    rx, rz = slam_to_cm(x, z)
    print(f"  [{total:3d}] raw slam=({x:.4f}, {z:.4f})  room=({rx:.1f}, {rz:.1f})cm")

# ── Phase 2: compute offset ─────────────────────────────────────────────────
avg_x = sum(s[0] for s in samples) / len(samples)
avg_z = sum(s[1] for s in samples) / len(samples)

true_x, true_z = MARKER_SLAM_POS[ANCHOR_MARKER]

offset_x = true_x - avg_x
offset_z = true_z - avg_z

print()
print(f"{'='*60}")
print(f"  Startup position (avg of {WARMUP_FRAMES} frames):")
print(f"    SLAM reported: ({avg_x:.4f}, {avg_z:.4f})")
print(f"    M{ANCHOR_MARKER} true pos:  ({true_x:.4f}, {true_z:.4f})")
print(f"    Offset:        ({offset_x:.4f}, {offset_z:.4f})")
print()

# Verify: apply offset to avg position → should equal true marker pos
check_x = avg_x + offset_x
check_z = avg_z + offset_z
rx, rz  = slam_to_cm(check_x, check_z)
print(f"  After offset → slam=({check_x:.4f}, {check_z:.4f})")
print(f"                 room=({rx:.1f}, {rz:.1f})cm")
print(f"  Expected room: M{ANCHOR_MARKER} = (125, 800)cm  [roughly]")
print(f"{'='*60}")
print()
print(f"  Now move around — watching corrected positions for 60s ...")
print()
print(f"  {'#':<5} {'slam_x':>10} {'slam_z':>10}  {'room_x':>8} {'room_y':>8}  {'bounds'}")
print(f"  {'-'*55}")

# ── Phase 3: live corrected output ─────────────────────────────────────────
t_start = time.time()
count   = 0

while time.time() - t_start < 60:
    try:
        msg = json.loads(sub.recv_string())
    except zmq.Again:
        print("  [TIMEOUT]")
        continue

    if not msg.get("ok"):
        print(f"  {'LOST':<5}")
        continue

    count += 1
    cx = msg["x"] + offset_x
    cz = msg["z"] + offset_z
    rx, rz = slam_to_cm(cx, cz)
    in_bounds = 0 <= rx <= ROOM_W and 0 <= rz <= ROOM_H
    print(f"  {count:<5} {cx:>10.4f} {cz:>10.4f}  {rx:>8.1f} {rz:>8.1f}  {'IN' if in_bounds else 'OUT'}")

print()
print("  Done. If room coords look right → Option 3 works.")
sub.close()
ctx.term()
