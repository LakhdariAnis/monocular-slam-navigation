"""
test_transform_static.py — Offline test of the SLAM-to-room coordinate transform.

Loads calibration.json and MarkerPositions.txt, runs each marker's SLAM
coordinates through the exact same transform used in zmq_bridge.py, and
compares the output to the expected room coordinates.

No ZMQ, no SLAM, no server needed.

Usage:
    python3 test_transform_static.py
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
PROJECT_DIR = SCRIPT_DIR.parent
CALIBRATION_FILE = PROJECT_DIR / "webui" / "data" / "calibration.json"
MARKER_FILE = PROJECT_DIR / "data" / "MarkerPositions.txt"

# ── Room layout (ground truth) ─────────────────────────────────────────────────
ROOM_W = 400  # cm
ROOM_H = 800  # cm

EXPECTED_ROOM_COORDS = {
    0: (332.0, 0.0),
    4: (125.0, 800.0),
    6: (400.0, 523.0),
    7: (0.0, 373.0),
}


# ── Transform function — copied exactly from zmq_bridge.py ─────────────────────
def slam_to_cm(slam_x, slam_z, H):
    pt = np.float32([[[slam_x, slam_z]]])
    result = cv2.perspectiveTransform(pt, np.float32(H))
    return float(result[0][0][0]), float(result[0][0][1])


# ── Load data ──────────────────────────────────────────────────────────────────
def load_calibration():
    if not CALIBRATION_FILE.exists():
        print(f"ERROR: calibration file not found: {CALIBRATION_FILE}")
        return None
    return json.loads(CALIBRATION_FILE.read_text())


def load_marker_positions():
    """Parse MarkerPositions.txt → dict of {marker_id: (x, z)}"""
    markers = {}
    if not MARKER_FILE.exists():
        print(f"ERROR: marker file not found: {MARKER_FILE}")
        return markers
    for line in MARKER_FILE.read_text().strip().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        mid = int(parts[0])
        x = float(parts[1])
        z = float(parts[3])  # SLAM z is the 4th column (index 3)
        markers[mid] = (x, z)
    return markers


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    cal = load_calibration()
    if cal is None:
        return

    H = np.float32(cal["transform_matrix"])
    marker_positions_slam = {int(k): v for k, v in cal.get("marker_positions_slam", {}).items()}
    marker_positions_real = {int(k): v for k, v in cal.get("marker_positions_real_cm", {}).items()}

    # Also load from MarkerPositions.txt for reference
    file_markers = load_marker_positions()

    print("=" * 76)
    print("SLAM → Room Coordinate Transform Test (Static)")
    print("=" * 76)
    print(f"Calibration file:  {CALIBRATION_FILE}")
    print(f"Marker file:       {MARKER_FILE}")
    print(f"Calibration source: {cal.get('source', 'unknown')}")
    print(f"Validation error:   {cal.get('validation_error_cm', 'N/A')} cm")
    print(f"Room dimensions:    {ROOM_W} x {ROOM_H} cm")
    print()

    # ── Test markers that exist in both calibration and expected ────────────
    test_ids = sorted(set(marker_positions_slam.keys()) & set(EXPECTED_ROOM_COORDS.keys()))

    if not test_ids:
        print("ERROR: No markers found in both calibration and expected coords!")
        return

    print(f"Testing {len(test_ids)} markers: {test_ids}")
    print("-" * 76)
    print(f"{'Marker':<8} {'SLAM (x,z)':<22} {'Room (x,y)':<22} {'Expected':<22} {'Error (cm)'}")
    print("-" * 76)

    errors = []

    for mid in test_ids:
        slam_x, slam_z = marker_positions_slam[mid]
        expected_x, expected_y = EXPECTED_ROOM_COORDS[mid]

        # Apply the exact same transform as zmq_bridge.py
        room_x, room_y = slam_to_cm(slam_x, slam_z, H)

        # Compute error
        err = math.sqrt((room_x - expected_x) ** 2 + (room_y - expected_y) ** 2)
        errors.append(err)

        print(
            f"M{mid:<7} "
            f"({slam_x:>8.4f}, {slam_z:>8.4f})  "
            f"({room_x:>7.1f}, {room_y:>7.1f})  "
            f"({expected_x:>7.1f}, {expected_y:>7.1f})  "
            f"{err:>8.2f}"
        )

    print("-" * 76)

    # ── Summary ────────────────────────────────────────────────────────────
    avg_err = sum(errors) / len(errors)
    max_err = max(errors)
    print()
    print(f"Average error:  {avg_err:.2f} cm")
    print(f"Max error:      {max_err:.2f} cm")
    print()

    # ── Also test markers from MarkerPositions.txt that are NOT calibration markers ──
    extra_ids = sorted(set(file_markers.keys()) - set(marker_positions_slam.keys()))
    if extra_ids:
        print(f"Extra markers in MarkerPositions.txt (not in calibration): {extra_ids}")
        print(f"{'Marker':<8} {'SLAM (x,z)':<22} {'Room (x,y)':<22} {'In bounds?'}")
        print("-" * 76)
        for mid in extra_ids:
            slam_x, slam_z = file_markers[mid]
            room_x, room_y = slam_to_cm(slam_x, slam_z, H)
            in_bounds = 0 <= room_x <= ROOM_W and 0 <= room_y <= ROOM_H
            print(
                f"M{mid:<7} "
                f"({slam_x:>8.4f}, {slam_z:>8.4f})  "
                f"({room_x:>7.1f}, {room_y:>7.1f})  "
                f"{'YES' if in_bounds else 'NO'}"
            )
        print()

    # ── Cross-check: calibration SLAM coords vs MarkerPositions.txt ────────
    print("Cross-check: calibration.json SLAM coords vs MarkerPositions.txt")
    print("-" * 76)
    for mid in test_ids:
        cal_slam = marker_positions_slam[mid]
        if mid in file_markers:
            file_slam = file_markers[mid]
            match = (abs(cal_slam[0] - file_slam[0]) < 0.001 and
                     abs(cal_slam[1] - file_slam[1]) < 0.001)
            status = "MATCH" if match else "MISMATCH"
            print(f"  M{mid}: cal=({cal_slam[0]:.6f}, {cal_slam[1]:.6f})  "
                  f"file=({file_slam[0]:.6f}, {file_slam[1]:.6f})  [{status}]")
        else:
            print(f"  M{mid}: in calibration but NOT in MarkerPositions.txt")
    print()
    print("Done.")


if __name__ == "__main__":
    main()
