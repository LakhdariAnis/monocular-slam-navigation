#!/usr/bin/env python3
"""
bridge.py — ZMQ JPEG subscriber → POSIX shared memory writer
Receives 640x480 JPEG frames from the Pi, decodes to BGR, writes to shm.

Shared memory layout (header + frame):
  [0:8]   uint64  sequence number (little-endian)
  [8:16]  double  timestamp (seconds since epoch, little-endian)
  [16:]   uint8   raw BGR frame (640 * 480 * 3 = 921600 bytes)

Total: 921616 bytes
"""

import time
import struct
import argparse
import numpy as np
import cv2
import zmq
from multiprocessing import shared_memory

# ── Constants ────────────────────────────────────────────────────────────────
WIDTH, HEIGHT, CHANNELS = 640, 480, 3
FRAME_BYTES = WIDTH * HEIGHT * CHANNELS          # 921 600
HEADER_BYTES = 8 + 8                             # seq (u64) + ts (f64)
SHM_SIZE = HEADER_BYTES + FRAME_BYTES            # 921 616
SHM_NAME = "orbframe"


def main(zmq_addr: str, show: bool) -> None:
    # ── Shared memory ────────────────────────────────────────────────────────
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        print(f"[bridge] Created shared memory '{SHM_NAME}' ({SHM_SIZE} bytes)")
    except FileExistsError:
        old = shared_memory.SharedMemory(name=SHM_NAME, create=False, size=SHM_SIZE)
        old.unlink()
        old.close()
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        print(f"[bridge] Recreated shared memory '{SHM_NAME}'")

    buf = shm.buf
    buf[:] = b"\x00" * SHM_SIZE  # zero memory on startup

    # ── ZMQ ─────────────────────────────────────────────────────────────────
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVHWM, 2)          # drop stale frames, don't queue
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(zmq_addr)
    print(f"[bridge] Connected to {zmq_addr}")

    seq = int(time.time() * 30) % (2**32)
    t_last = time.monotonic()
    try:
        while True:
            # Receive raw JPEG bytes
            jpeg_bytes = sock.recv()
            ts = time.time() 

            # Decode JPEG → BGR numpy array
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

            if frame is None:
                print("[bridge] Warning: failed to decode frame, skipping")
                continue

            if frame.shape != (HEIGHT, WIDTH, CHANNELS):
                frame = cv2.resize(frame, (WIDTH, HEIGHT))

            # Write header then frame into shared memory atomically enough
            # (C++ reader checks seq change to detect new frame)
            # Write ts+frame first, seq last as commit signal
            struct.pack_into("<d", buf, 8, ts)
            buf[HEADER_BYTES:HEADER_BYTES + FRAME_BYTES] = frame.tobytes()
            struct.pack_into("<Q", buf, 0, seq)
            seq += 1

            # FPS display
            now = time.monotonic()
            if now - t_last >= 2.0:
                print(f"[bridge] seq={seq}  ts={ts:.3f}")
                t_last = now

            if show:
                cv2.imshow("bridge preview", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[bridge] Shutting down")
    finally:
        buf.release()
        shm.close()
        shm.unlink()
        sock.close()
        ctx.term()
        if show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ZMQ→SHM bridge for ORB-SLAM3")
    parser.add_argument("--zmq", default="tcp://raspberrypi.local:5555",
                        help="ZMQ publisher address (default: tcp://raspberrypi.local:5555)")
    parser.add_argument("--show", action="store_true",
                        help="Show live preview window")
    args = parser.parse_args()
    main(args.zmq, args.show)
