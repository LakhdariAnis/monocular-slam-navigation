#!/usr/bin/env python3
"""
calibrate_cli.py — Standalone CLI calibration for autonomous car.
Replaces the web UI /calibrate flow. Same math, no browser needed.

Usage:
    python3 calibrate_cli.py photo1.jpg photo2.jpg [photo3.jpg ...]

Output:
    Overwrites ~/autonomous_car/webui/data/calibration.json

Tips:
    - Each photo should show at least 2 of the 4 markers (M0, M4, M6, M7)
    - Cover all pairs: e.g. M0+M7, M0+M6, M4+M7, M4+M6
    - Markers must be well-lit, flat, and facing the camera
    - 4-6 photos is enough
"""

import sys
import json
import argparse
from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent  # ~/autonomous_car/test/ → ~/autonomous_car/
WEBUI_DIR    = PROJECT_ROOT / "webui"
CAL_OUT      = WEBUI_DIR / "data" / "calibration.json"

# Add webui to path so we can import calibrate.py directly
sys.path.insert(0, str(WEBUI_DIR))

try:
    from calibrate import detect_markers_in_image, compute_transform_from_photos
except ImportError as e:
    print(f"[ERROR] Could not import calibrate.py from {WEBUI_DIR}: {e}")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate SLAM→room transform from photos (no web UI needed)"
    )
    parser.add_argument(
        "photos",
        nargs="+",
        help="Paths to photo files (JPEG or PNG)"
    )
    parser.add_argument(
        "--out",
        default=str(CAL_OUT),
        help=f"Output calibration.json path (default: {CAL_OUT})"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't save, just print results"
    )
    args = parser.parse_args()

    out_path = Path(args.out)
    photo_paths = [Path(p) for p in args.photos]

    # ── Validate inputs ────────────────────────────────────────────────────────
    for p in photo_paths:
        if not p.exists():
            print(f"[ERROR] File not found: {p}")
            sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  Autonomous Car — CLI Calibration")
    print(f"{'='*60}")
    print(f"  Photos:  {len(photo_paths)}")
    print(f"  Output:  {out_path}")
    print(f"{'='*60}\n")

    # ── Step 1: detect markers in each photo ──────────────────────────────────
    photo_results = []
    total_markers_found = set()

    for i, photo_path in enumerate(photo_paths):
        print(f"[{i+1}/{len(photo_paths)}] Processing {photo_path.name} ...")
        img_bytes = photo_path.read_bytes()

        try:
            markers, rejected, _, cam_source, (w, h) = detect_markers_in_image(img_bytes)
        except Exception as e:
            print(f"  [ERROR] Failed to process {photo_path.name}: {e}")
            continue

        found_ids = sorted(markers.keys())
        print(f"  Resolution: {w}x{h}  |  Camera: {cam_source}")

        if found_ids:
            print(f"  Markers found: {[f'M{mid}' for mid in found_ids]}")
            for mid in found_ids:
                tvec = markers[mid]["tvec"]
                import numpy as np
                dist = np.linalg.norm(tvec) * 100
                print(f"    M{mid}: distance={dist:.1f}cm  tvec=({tvec[0]*100:.1f}, {tvec[1]*100:.1f}, {tvec[2]*100:.1f})cm")
        else:
            print(f"  [WARN] No usable markers found in this photo")

        if rejected:
            print(f"  Rejected (too small): {[f'M{mid}' for mid in rejected]}")

        if markers:
            photo_results.append(markers)
            total_markers_found.update(found_ids)

    print(f"\n  Total unique markers across all photos: {sorted(total_markers_found)}")

    if len(photo_results) == 0:
        print("\n[ERROR] No photos yielded any markers. Check lighting and marker visibility.")
        sys.exit(1)

    target = {0, 4, 6, 7}
    missing = target - total_markers_found
    if missing:
        print(f"\n[WARN] Missing markers from target set: {[f'M{m}' for m in sorted(missing)]}")
        print(f"       Calibration may be less accurate.")

    # ── Step 2: compute transform ──────────────────────────────────────────────
    print(f"\n  Computing SLAM→room transform ...")

    try:
        result = compute_transform_from_photos(photo_results)
    except Exception as e:
        print(f"\n[ERROR] Transform computation failed: {e}")
        sys.exit(1)

    # ── Step 3: print results ──────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"  Results")
    print(f"{'='*60}")
    print(f"  Transform source:    {result.get('transform_source', '?')}")
    print(f"  Markers used:        {result.get('markers_used', '?')}")
    print(f"  Scale:               {result.get('scale_cm_per_slam_unit', 0):.2f} cm/SLAM unit")
    print(f"  Validation error:    {result.get('validation_error_cm', 0):.2f} cm")

    print(f"\n  Marker positions (solved):")
    real_cm = result.get("marker_positions_real_cm", {})
    slam_pos = result.get("marker_positions_slam", {})
    for mid_str in sorted(real_cm.keys(), key=lambda x: int(x)):
        rx, ry = real_cm[mid_str]
        sx, sz = slam_pos.get(mid_str, [0, 0])
        print(f"    M{mid_str}: room=({rx:.1f}, {ry:.1f})cm  slam=({sx:.4f}, {sz:.4f})")

    val_err = result.get("validation_error_cm", 999)
    print(f"\n  {'✓ GOOD' if val_err < 20 else '⚠ HIGH ERROR'} — Validation error: {val_err:.2f}cm")
    if val_err >= 20:
        print(f"  Consider retaking photos with better marker visibility.")
    if val_err >= 40:
        print(f"  [ERROR] Error too high (>=40cm). Do not save — retake photos.")
        sys.exit(1)

    # ── Step 4: save ──────────────────────────────────────────────────────────
    if args.dry_run:
        print(f"\n  [DRY RUN] Not saving. Would write to {out_path}")
        print(f"\n  Transform matrix:")
        for row in result["transform_matrix"]:
            print(f"    {[f'{v:.4f}' for v in row]}")
    else:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, indent=2))
        print(f"\n  Saved → {out_path}")

    print(f"\n  Next step: run python3 test/test_transform_static.py to verify.\n")


if __name__ == "__main__":
    main()
