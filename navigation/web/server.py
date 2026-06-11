#!/usr/bin/env python3
"""
server.py — FastAPI web dashboard for autonomous car navigation.

Usage:
    python3 navigation/web/server.py --mode test     # mock navigator
    python3 navigation/web/server.py --mode real     # real hardware
"""

import argparse
import json
import os
import sys
import re
import io

import tempfile
import shutil
from typing import List

import numpy as np
from fastapi import FastAPI, Request, UploadFile
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

import navigation.navigator as nav

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
NAV_DIR       = os.path.dirname(BASE_DIR)                       # navigation/
PROJECT_DIR   = os.path.dirname(NAV_DIR)                        # autonomous_car/
STATIONS_FILE = os.path.join(NAV_DIR, "stations.json")
MAPPOINTS     = os.path.join(PROJECT_DIR, "data", "MapPoints.txt")
INDEX_HTML    = os.path.join(BASE_DIR, "index.html")
CAL_FILE      = os.path.join(PROJECT_DIR, "data", "calibration.json")


# ─── Phase-tracking stdout wrapper ───────────────────────────────────────────
_PHASE_RE = re.compile(
    r"\[Phase (\d)\]"
    r"|\[nav\] departing"
    r"|\[nav\] arrived"
    r"|\[depart\]"
)

class _PhaseCapture(io.TextIOBase):
    """Wraps stdout, intercepts navigator prints to update nav phase state."""
    def __init__(self, real_stdout):
        self._real = real_stdout

    def write(self, s):
        self._real.write(s)
        m = _PHASE_RE.search(s)
        if m:
            if m.group(1):                     # [Phase 0] … [Phase 3]
                pass  # navigator handles its own phase state
            elif "departing" in s:
                pass
            elif "arrived" in s:
                pass
        return len(s)

    def flush(self):
        self._real.flush()

    # Forward attributes so libs don't crash
    def fileno(self):          return self._real.fileno()
    def isatty(self):          return False
    @property
    def encoding(self):        return getattr(self._real, 'encoding', 'utf-8')


# ─── Convex hull (cached) ────────────────────────────────────────────────────
_hull_cache = None

def _compute_hull():
    """Compute 2D convex hull on (x, z) from MapPoints.txt."""
    global _hull_cache
    if _hull_cache is not None:
        return _hull_cache
    try:
        from scipy.spatial import ConvexHull
        pts = np.loadtxt(MAPPOINTS)
        px, pz = pts[:, 0], pts[:, 2]
        pts2d = np.column_stack([px, pz])
        hull = ConvexHull(pts2d)
        hull_pts = pts2d[hull.vertices].tolist()
        hull_pts.append(hull_pts[0])       # close polygon
        _hull_cache = hull_pts
        return _hull_cache
    except Exception as e:
        print(f"[server] hull error: {e}")
        return []


# ─── FastAPI app ──────────────────────────────────────────────────────────────
app = FastAPI()

# Enable CORS for all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/", response_class=HTMLResponse)
async def root():
    return FileResponse(INDEX_HTML, media_type="text/html")


@app.get("/state")
async def get_state():
    return nav.get_state()


@app.get("/stations")
async def get_stations():
    with open(STATIONS_FILE) as f:
        return json.load(f)


@app.get("/map_hull")
async def map_hull():
    hull = _compute_hull()
    return {"hull": hull}


@app.get("/calibration")
async def get_calibration():
    cal_path = os.path.join(PROJECT_DIR, "data", "calibration.json")
    try:
        with open(cal_path) as f:
            return json.load(f)
    except FileNotFoundError:
        return {"error": "calibration.json not found. Run tools/calibrate_cli.py with physical markers."}
    except Exception as e:
        return {"error": str(e)}


@app.post("/calibrate")
async def run_calibration(photos: List[UploadFile]):
    """
    Accept uploaded photos, run ArUco detection + transform computation,
    save result to data/calibration.json if error < 40cm.
    Returns full result dict with per-photo detection info.
    """
    import sys as _sys
    _tools = os.path.join(NAV_DIR, "tools")
    if _tools not in _sys.path:
        _sys.path.insert(0, _tools)

    try:
        from calibrate import detect_markers_in_image, compute_transform_from_photos
    except ImportError as e:
        return JSONResponse({"error": f"Cannot import calibrate.py: {e}"}, status_code=500)

    photo_results = []
    per_photo = []
    total_markers = set()
    tmp_dir = tempfile.mkdtemp()

    try:
        for upload in photos:
            img_bytes = await upload.read()
            try:
                markers, rejected, _, cam_source, (w, h) = detect_markers_in_image(img_bytes)
            except Exception as e:
                per_photo.append({
                    "filename": upload.filename,
                    "error": str(e),
                    "markers": []
                })
                continue

            found_ids = sorted(markers.keys())
            per_photo.append({
                "filename": upload.filename,
                "width": w,
                "height": h,
                "cam_source": cam_source,
                "markers": [f"M{mid}" for mid in found_ids],
                "rejected": [f"M{mid}" for mid in rejected]
            })

            if markers:
                photo_results.append(markers)
                total_markers.update(found_ids)

        if not photo_results:
            return JSONResponse({
                "success": False,
                "error": "No markers found in any photo. Check lighting and marker visibility.",
                "per_photo": per_photo
            })

        result = compute_transform_from_photos(photo_results)
        result["per_photo"] = per_photo
        result["total_markers_found"] = [f"M{m}" for m in sorted(total_markers)]

        val_err = result.get("validation_error_cm", 999)
        if val_err < 40:
            with open(CAL_FILE, "w") as f:
                json.dump({k: v for k, v in result.items() if k not in ("per_photo", "total_markers_found")}, f, indent=2)
            result["saved"] = True
        else:
            result["saved"] = False
            result["save_skipped_reason"] = f"Validation error {val_err:.1f}cm >= 40cm threshold"

        result["success"] = True
        return result

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


@app.post("/go")
async def go(request: Request):
    body = await request.json()
    target = body.get("target", "")

    state = nav.get_state()
    if state.get("running"):
        return {"status": "busy"}

    nav.navigate_to(target)
    return {"status": "started", "target": target}


@app.post("/stop")
async def stop():
    nav.stop()
    return {"status": "stopped"}


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Autonomous car web dashboard")
    parser.add_argument("--mode", choices=["test", "real"], default="test")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    # Install phase-capture stdout wrapper
    sys.stdout = _PhaseCapture(sys.__stdout__)

    # Initialize the navigator module
    nav.init(STATIONS_FILE, args.mode)

    # In test mode, force mock pose to "start" station position
    if args.mode == "test":
        try:
            with open(STATIONS_FILE) as f:
                sts = json.load(f)
            if "start" in sts:
                with nav._mock_lock:
                    nav._mock_pose = (sts["start"]["x"], sts["start"]["z"])
                    # Face target based on station orientation (-Z wall)
                    nav._mock_heading = 180.0
                nav._docked_at = "start"
                print("[server] Forced test start position to 'start'")
        except Exception:
            pass

    print(f"[server] mode={args.mode}  port={args.port}")
    print(f"[server] http://localhost:{args.port}")

    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
