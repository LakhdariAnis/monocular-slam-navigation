#!/usr/bin/env python3
"""
slam_zmq.py — Launches slam_reader in localization mode and publishes
              its pose output on ZMQ PUB tcp://*:5557

Usage (from scripts/build/):
    python3 slam_zmq.py

Publishes JSON on port 5557:
    {"seq":123, "ts":456.78, "ok":true,  "x":-0.045, "y":-0.081, "z":0.430, "reset":false}
    {"seq":123, "ts":0.0,    "ok":false, "x":null,   "y":null,   "z":null,   "reset":false}  # lost
    {"seq":-1,  "ts":0.0,    "ok":false, "x":null,   "y":null,   "z":null,   "reset":true}   # MAP RESET

After a map reset, every subsequent pose is published with reset=True until
navigator.py sends the string "ACK" on the REP socket tcp://*:5558.
This means the navigator — not a countdown — decides when it is safe to resume.
"""

import subprocess
import os
import re
import json
import signal
import math
import threading
import zmq

ZMQ_PUB_PORT = 5557   # poses go out here
ZMQ_ACK_PORT = 5558   # navigator sends "ACK" here to clear the reset latch

# ── Paths ─────────────────────────────────────────────────────────────────
BUILD_DIR   = os.path.expanduser("~/autonomous_car/scripts/build")
SLAM_READER = os.path.join(BUILD_DIR, "slam_reader")
VOCAB       = os.path.expanduser("~/autonomous_car/ORB_SLAM3/Vocabulary/ORBvoc.txt")
CONFIG      = os.path.expanduser("~/autonomous_car/config/picam.yaml")
ATLAS_NAME  = "room_map"
DATA_DIR    = os.path.expanduser("~/autonomous_car/data")

# ── Tuning ────────────────────────────────────────────────────────────────
# Euclidean XZ jump that signals a silent coordinate-frame reset.
JUMP_THRESHOLD_M = 0.5

# Heartbeat: print a log line every N published poses (~0.3 s at 30 fps).
PRINT_EVERY = 10

# ── Regex parsers ─────────────────────────────────────────────────────────
RE_TRACKED = re.compile(
    r'\[SLAM\]\s+seq=(\d+)\s+ts=([\d.]+)\s+'
    r'x=([-\d.]+)\s+y=([-\d.]+)\s+z=([-\d.]+)'
)
RE_LOST = re.compile(
    r'\[SLAM\]\s+seq=(\d+)\s+TRACKING LOST'
)
# Anchored to the single ORB-SLAM3 trigger line; LM follow-ups are excluded.
RE_SLAM_RESET = re.compile(
    r'SYSTEM\s*->\s*Reseting active map',
    re.IGNORECASE
)


def xz_distance(a, b):
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["z"] - b["z"]) ** 2)


def ack_listener(ack_socket, reset_event, stop_event):
    """
    Background thread: waits for the navigator to send "ACK" on the REP
    socket, then clears the reset latch (reset_event).
    """
    while not stop_event.is_set():
        try:
            # 500 ms poll so the thread exits cleanly on shutdown
            if ack_socket.poll(500):
                msg = ack_socket.recv_string()
                if msg.strip().upper() == "ACK":
                    reset_event.clear()
                    print("[slam_zmq] ✓ ACK received — reset latch cleared, "
                          "resuming normal poses.")
                ack_socket.send_string("OK")
        except zmq.ZMQError:
            break


def main():
    # ── ZMQ setup ─────────────────────────────────────────────────────────
    ctx = zmq.Context()

    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{ZMQ_PUB_PORT}")

    ack_rep = ctx.socket(zmq.REP)
    ack_rep.bind(f"tcp://*:{ZMQ_ACK_PORT}")

    print(f"[slam_zmq] ZMQ PUB on tcp://*:{ZMQ_PUB_PORT}")
    print(f"[slam_zmq] ZMQ ACK REP on tcp://*:{ZMQ_ACK_PORT}  "
          f"(navigator sends 'ACK' here to resume after reset)")

    # reset_event: SET = map reset active, navigator must not use poses.
    # Cleared only when navigator sends ACK.
    reset_event = threading.Event()
    stop_event  = threading.Event()

    ack_thread = threading.Thread(
        target=ack_listener,
        args=(ack_rep, reset_event, stop_event),
        daemon=True,
    )
    ack_thread.start()

    # ── Launch slam_reader ────────────────────────────────────────────────
    cmd = [
        "prime-run", SLAM_READER, VOCAB, CONFIG,
        "--loc", ATLAS_NAME,
        "--output-dir", DATA_DIR,
        
    ]
    print(f"[slam_zmq] Launching: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        cwd=BUILD_DIR,
    )

    def handle_sig(signum, frame):
        proc.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    print(f"[slam_zmq] slam_reader PID {proc.pid}")
    print(f"[slam_zmq] Waiting for SLAM poses ...\n")

    last_valid_pose = None   # most recent clean tracked pose
    consec_lost     = 0
    count           = 0

    for line in proc.stdout:
        line = line.rstrip('\n')

        # ── Primary reset signal: ORB-SLAM3 log line ──────────────────────
        if RE_SLAM_RESET.search(line) and not reset_event.is_set():
            print(line)
            reset_event.set()
            last_valid_pose = None
            # Publish bare reset notification (no pose data)
            pub.send_string(json.dumps({
                "seq": -1, "ts": 0.0, "ok": False,
                "x": None, "y": None, "z": None,
                "reset": True,
            }))
            print(
                f"\n⚠️  MAP RESET DETECTED\n"
                f"    All poses will carry reset=True until navigator sends ACK\n"
                f"    on tcp://*:{ZMQ_ACK_PORT}\n"
            )
            continue

        # ── Tracking-lost lines ───────────────────────────────────────────
        m = RE_LOST.search(line)
        if m:
            seq = int(m.group(1))
            consec_lost += 1
            if consec_lost == 1:
                print(f"[slam_zmq] ⚠ TRACKING LOST  seq={seq}")
            elif consec_lost % 10 == 0:
                print(f"[slam_zmq] ⚠ STILL LOST  seq={seq}  ({consec_lost} frames)")
            pub.send_string(json.dumps({
                "seq": seq, "ts": 0.0, "ok": False,
                "x": None, "y": None, "z": None,
                "reset": reset_event.is_set(),   # propagate latch state
            }))
            count += 1
            continue

        # ── Tracked-pose lines ────────────────────────────────────────────
        m = RE_TRACKED.search(line)
        if m:
            seq  = int(m.group(1))
            ts   = float(m.group(2))
            pose = {
                "x": float(m.group(3)),
                "y": float(m.group(4)),
                "z": float(m.group(5)),
            }

            # Secondary reset signal: large XZ jump
            if (last_valid_pose is not None
                    and not reset_event.is_set()
                    and xz_distance(last_valid_pose, pose) >= JUMP_THRESHOLD_M):
                reset_event.set()
                last_valid_pose = None
                pub.send_string(json.dumps({
                    "seq": seq, "ts": ts, "ok": False,
                    "x": None, "y": None, "z": None,
                    "reset": True,
                }))
                print(
                    f"\n⚠️  MAP RESET DETECTED (pose jump, seq={seq})\n"
                    f"    All poses will carry reset=True until navigator sends ACK\n"
                    f"    on tcp://*:{ZMQ_ACK_PORT}\n"
                )

            in_reset = reset_event.is_set()

            if in_reset:
                # Publish the new (untrusted) coordinates but always flag reset=True.
                # Navigator must NOT use these for navigation — they are in an
                # unknown frame. The flag stays True until ACK is received.
                msg = {
                    "seq":   seq,
                    "ts":    ts,
                    "ok":    True,
                    "x":     pose["x"],
                    "y":     pose["y"],
                    "z":     pose["z"],
                    "reset": True,
                }
            else:
                msg = {
                    "seq":   seq,
                    "ts":    ts,
                    "ok":    True,
                    "x":     pose["x"],
                    "y":     pose["y"],
                    "z":     pose["z"],
                    "reset": False,
                }
                last_valid_pose = pose   # only advance on confirmed clean frames

            pub.send_string(json.dumps(msg))
            count += 1
            consec_lost = 0

            if count % PRINT_EVERY == 0:
                flag = " [RESET — waiting for ACK]" if in_reset else ""
                print(f"[slam_zmq] {count} poses | "
                      f"x={msg['x']:.4f} z={msg['z']:.4f}"
                      f"{flag}")
            continue

        # ── Pass through everything else ──────────────────────────────────
        print(line)

    stop_event.set()
    proc.wait()
    pub.close()
    ack_rep.close()
    ctx.term()
    print(f"\n[slam_zmq] Done. {count} total poses published.")


if __name__ == "__main__":
    main()
