"""
test_transform_live.py — Live test of the SLAM-to-room coordinate transform.

Subscribes to ZMQ port 5557 (same as zmq_bridge.py), applies the exact same
perspectiveTransform, and prints per-pose diagnostics. Runs for 60 seconds
then prints a summary.

Requirements: SLAM must be running and publishing on port 5557.

Usage:
    python3 test_transform_live.py
"""

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import zmq

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
PROJECT_DIR = SCRIPT_DIR.parent
CALIBRATION_FILE = PROJECT_DIR / "webui" / "data" / "calibration.json"

# ── Room bounds ────────────────────────────────────────────────────────────────
ROOM_W = 400  # cm
ROOM_H = 800  # cm

DURATION_SEC = 60


# ── Transform function — copied exactly from zmq_bridge.py ─────────────────────
def slam_to_cm(slam_x, slam_z, H):
    pt = np.float32([[[slam_x, slam_z]]])
    result = cv2.perspectiveTransform(pt, np.float32(H))
    return float(result[0][0][0]), float(result[0][0][1])


# ── Heading function — copied exactly from zmq_bridge.py ───────────────────────
def heading_from_R(R_flat):
    """Extract yaw (heading) from 9-value row-major rotation matrix."""
    R = np.array(R_flat).reshape(3, 3)
    heading_rad = math.atan2(R[0, 2], R[2, 2])
    return math.degrees(heading_rad)


# ── Load calibration ──────────────────────────────────────────────────────────
def load_calibration():
    if not CALIBRATION_FILE.exists():
        print(f"ERROR: calibration file not found: {CALIBRATION_FILE}")
        return None
    return json.loads(CALIBRATION_FILE.read_text())


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    cal = load_calibration()
    if cal is None:
        return

    H = np.float32(cal["transform_matrix"])

    print("=" * 80)
    print("SLAM → Room Coordinate Transform Test (Live)")
    print("=" * 80)
    print(f"Calibration: {CALIBRATION_FILE}")
    print(f"Source: {cal.get('source', 'unknown')}")
    print(f"Room: {ROOM_W} x {ROOM_H} cm")
    print(f"Duration: {DURATION_SEC}s")
    print(f"Subscribing to tcp://localhost:5557 ...")
    print()

    # ZMQ setup — same as zmq_bridge.py
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://localhost:5557")
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 2000)  # 2s timeout, same as zmq_bridge.py

    # Stats
    total_received = 0
    ok_count = 0
    not_ok_count = 0
    in_bounds_count = 0
    out_of_bounds_count = 0
    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")
    tracking_lost_count = 0

    start_time = time.time()

    print(f"{'#':<6} {'Time':<8} {'ok':<5} {'SLAM x':>10} {'SLAM z':>10}  "
          f"{'Room x':>8} {'Room y':>8} {'Heading':>8} {'Bounds'}")
    print("-" * 80)

    try:
        while time.time() - start_time < DURATION_SEC:
            try:
                raw = sub.recv_string()
            except zmq.Again:
                # Timeout — SLAM not publishing
                elapsed = time.time() - start_time
                print(f"{'---':<6} {elapsed:>7.1f}s  TIMEOUT - no SLAM data (waiting...)")
                tracking_lost_count += 1
                continue
            except zmq.ZMQError as e:
                print(f"[ZMQ error] {e}")
                time.sleep(0.5)
                continue

            try:
                pose_msg = json.loads(raw)
            except json.JSONDecodeError as e:
                print(f"[JSON error] {e}")
                continue

            total_received += 1
            elapsed = time.time() - start_time
            ok = pose_msg.get("ok", False)

            if not ok:
                not_ok_count += 1
                tracking_lost_count += 1
                print(f"{total_received:<6} {elapsed:>7.1f}s  ok=NO  (tracking lost)")
                continue

            ok_count += 1
            slam_x = pose_msg.get("x", 0.0)
            slam_z = pose_msg.get("z", 0.0)

            # Apply transform — exact same as zmq_bridge.py
            try:
                room_x, room_y = slam_to_cm(slam_x, slam_z, H)
            except Exception as e:
                print(f"{total_received:<6} {elapsed:>7.1f}s  TRANSFORM ERROR: {e}")
                continue

            # Heading
            heading = 0.0
            if "R" in pose_msg:
                try:
                    heading = heading_from_R(pose_msg["R"])
                except Exception:
                    pass

            # Bounds check
            in_bounds = 0 <= room_x <= ROOM_W and 0 <= room_y <= ROOM_H
            if in_bounds:
                in_bounds_count += 1
            else:
                out_of_bounds_count += 1

            # Track min/max
            min_x = min(min_x, room_x)
            max_x = max(max_x, room_x)
            min_y = min(min_y, room_y)
            max_y = max(max_y, room_y)

            bounds_str = "IN" if in_bounds else "OUT"
            print(
                f"{total_received:<6} {elapsed:>7.1f}s  ok=YES "
                f"{slam_x:>10.4f} {slam_z:>10.4f}  "
                f"{room_x:>8.1f} {room_y:>8.1f} {heading:>8.1f}° "
                f"{bounds_str}"
            )

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    # ── Summary ────────────────────────────────────────────────────────────
    duration = time.time() - start_time
    print()
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"Duration:             {duration:.1f}s")
    print(f"Total poses received: {total_received}")
    print(f"  ok=true:            {ok_count}")
    print(f"  ok=false:           {not_ok_count}")
    print(f"Tracking timeouts:    {tracking_lost_count}")
    print()
    if ok_count > 0:
        print(f"Room X range:         {min_x:.1f} — {max_x:.1f} cm  (room: 0–{ROOM_W})")
        print(f"Room Y range:         {min_y:.1f} — {max_y:.1f} cm  (room: 0–{ROOM_H})")
        print(f"In bounds:            {in_bounds_count}")
        print(f"Out of bounds:        {out_of_bounds_count}")
        if ok_count > 0:
            pct = in_bounds_count / ok_count * 100
            print(f"In-bounds rate:       {pct:.1f}%")
    else:
        print("No valid poses received — nothing to summarise.")
    print()
    print("Done.")

    sub.close()
    ctx.term()


if __name__ == "__main__":
    main()
