"""
calibrate.py — ArUco photo calibration logic
Detects markers in phone photos, runs solvePnP, computes SLAM→room transform.
"""

import cv2
import numpy as np
import json
import base64
import io
import struct
from datetime import datetime
from pathlib import Path
# ── Paths ──────────────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent.parent   # navigation/tools/ → navigation/ → autonomous_car/
DATA_DIR = PROJECT_ROOT / "data"
MARKER_POSITIONS_FILE = DATA_DIR / "MarkerPositions.txt"

# ── Constants ─────────────────────────────────────────────────────────────────
MARKER_SIZE = 0.184  # meters (18.4 cm)
ARUCO_DICT = cv2.aruco.DICT_4X4_50


def load_slam_marker_positions():
    """Load marker positions from data/MarkerPositions.txt."""
    positions = {}
    path = MARKER_POSITIONS_FILE
    if not path.exists():
        return {}
    
    try:
        with open(path, "r") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                parts = line.split()
                if len(parts) >= 4:
                    mid = int(parts[0])
                    x = float(parts[1])
                    z = float(parts[3]) # SLAM coords: x, y, z -> we use x and z for 2D plane
                    positions[mid] = (x, z)
    except Exception as e:
        print(f"[calibrate] Error loading {path}: {e}")
    
    return positions

# Default fallback if file is missing or empty
SLAM_MARKER_POSITIONS = {
    0: (-0.984894, -0.808958),
    4: (-0.002214,  1.765292),
    6: ( 0.554097, -0.451729),
    7: ( 0.227829, -8.329228),
}

# ── ArUco detector setup ───────────────────────────────────────────────────────
def _get_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


DETECTOR = _get_detector()

# ── EXIF focal length extraction ───────────────────────────────────────────────
def _extract_camera_matrix(img_bytes: bytes, w: int, h: int):
    """
    Try to extract focal length from EXIF. Falls back to estimate.
    Returns (cam_matrix, dist_coeffs, source_str)
    """
    cx, cy = w / 2.0, h / 2.0
    try:
        import piexif
        exif = piexif.load(img_bytes)
        exif_ifd = exif.get("Exif", {})
        focal_mm_tag = exif_ifd.get(piexif.ExifIFD.FocalLength)
        focal_35_tag = exif_ifd.get(piexif.ExifIFD.FocalLengthIn35mmFilm)

        if focal_35_tag and focal_35_tag > 0:
            # Use 35mm equivalent: sensor diagonal ~43.3mm for 35mm film
            # fx = focal_35mm * pixel_diagonal / 43.3
            diag_px = (w**2 + h**2) ** 0.5
            fx = fy = (focal_35_tag * diag_px) / 43.3
            source = f"exif_35mm_equiv({focal_35_tag}mm)"
        elif focal_mm_tag:
            num, den = focal_mm_tag if isinstance(focal_mm_tag, tuple) else (focal_mm_tag, 1)
            focal_mm = num / (den or 1)
            # Pixel 8 sensor width ~6.4mm at 12MP (4000px wide)
            # scale to actual image width
            sensor_w_mm = 6.4 * (4000 / max(w, 1))
            fx = fy = (focal_mm * w) / sensor_w_mm
            source = f"exif_focal({focal_mm:.1f}mm)"
        else:
            raise ValueError("no focal tag")
    except Exception:
        fx = fy = 1.2 * max(w, h)
        source = "estimated(no_exif)"

    cam_matrix = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1],
    ], dtype=np.float64)
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    return cam_matrix, dist_coeffs, source


# ── Core marker detection ──────────────────────────────────────────────────────
def detect_markers_in_image(img_bytes: bytes):
    """
    Detect ArUco markers in image bytes.
    Returns dict: { marker_id: {"corners": [[x,y]x4], "tvec": [x,y,z], "rvec": [x,y,z]} }
    plus rejected_ids, preview image as base64 JPEG and camera intrinsics source string.
    """
    nparr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Could not decode image")

    h, w = img.shape[:2]
    cam_matrix, dist_coeffs, cam_source = _extract_camera_matrix(img_bytes, w, h)

    # Detect
    corners, ids, _ = DETECTOR.detectMarkers(img)

    results = {}
    rejected_ids = []
    if ids is not None and len(ids) > 0:
        # 3D object points for marker corners (marker frame, Z=0)
        half = MARKER_SIZE / 2
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        for i, marker_id in enumerate(ids.flatten()):
            img_pts = corners[i][0].astype(np.float64)

            # --- Size Filtering ---
            # 1. Area filter
            area = cv2.contourArea(img_pts.astype(np.float32))
            
            # 2. Side length filter (average of 4 sides)
            p = img_pts
            sides = [
                np.linalg.norm(p[0] - p[1]),
                np.linalg.norm(p[1] - p[2]),
                np.linalg.norm(p[2] - p[3]),
                np.linalg.norm(p[3] - p[0])
            ]
            avg_side = sum(sides) / 4.0

            if area < 1500 or avg_side < 35:
                rejected_ids.append(int(marker_id))
                continue

            ret, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, cam_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if ret:
                results[int(marker_id)] = {
                    "corners": corners[i][0].tolist(),
                    "tvec": tvec.flatten().tolist(),
                    "rvec": rvec.flatten().tolist(),
                }

        # Draw detections on preview
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
            c = corners[i][0].mean(axis=0).astype(int)
            
            if mid in results:
                tvec = results[mid]["tvec"]
                dist = np.linalg.norm(tvec)
                cv2.putText(img, f"M{mid} {dist*100:.0f}cm",
                            (c[0]-30, c[1]-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            elif mid in rejected_ids:
                cv2.putText(img, f"M{mid} TOO SMALL",
                            (c[0]-30, c[1]-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Encode preview
    _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    preview_b64 = "data:image/jpeg;base64," + base64.b64encode(buf).decode()

    return results, rejected_ids, preview_b64, cam_source, (w, h)


# ── Multi-photo fusion ─────────────────────────────────────────────────────────
def _marker_distance_2d(tvec_a, tvec_b):
    """
    Distance between two markers in the camera XZ plane (horizontal), in meters.
    Camera X = right, Y = down, Z = depth.
    We use X and Z for horizontal layout.
    """
    return np.array([tvec_a[0] - tvec_b[0], tvec_a[2] - tvec_b[2]])


def fuse_photos(photo_results: list[dict]) -> dict:
    """
    photo_results: list of {marker_id: {corners, tvec, rvec}, ...} — one per photo.
    
    Builds a connected graph of markers using shared markers between photos as bridges.
    Returns: {marker_id: [x_cm, y_cm]} positions in a consistent real-world frame,
             plus the homography H mapping SLAM(x,z) → room cm.
    """
    # Build graph: for each photo, compute relative positions between all visible markers
    # Use the first photo's first marker as the origin

    # Step 1: collect all unique marker IDs
    all_ids = set()
    for photo in photo_results:
        all_ids.update(photo.keys())

    if len(all_ids) < 2:
        raise ValueError("Need at least 2 markers total across all photos")

    # Step 2: for each photo, compute pairwise relative positions (in camera frame → meters)
    # We'll build a position graph: marker_id → position in a shared 2D frame (cm)
    positions = {}  # marker_id → np.array([x, y]) in cm, shared frame

    # Process photos one by one, anchoring to already-placed markers
    for photo_idx, photo in enumerate(photo_results):
        marker_ids = list(photo.keys())
        if len(marker_ids) < 2:
            continue

        # Find which markers from this photo are already placed
        placed = {mid: positions[mid] for mid in marker_ids if mid in positions}
        unplaced = [mid for mid in marker_ids if mid not in positions]

        if not placed and photo_idx == 0:
            # First photo: place first marker at origin, second relative to it
            anchor_id = marker_ids[0]
            positions[anchor_id] = np.array([0.0, 0.0])
            placed = {anchor_id: positions[anchor_id]}
            unplaced = marker_ids[1:]

        # For each unplaced marker, compute position relative to a placed one
        for placed_id, placed_pos in placed.items():
            placed_tvec = np.array(photo[placed_id]["tvec"])
            for unplaced_id in unplaced:
                if unplaced_id in positions:
                    continue
                unplaced_tvec = np.array(photo[unplaced_id]["tvec"])
                # Relative offset in camera frame (X, Z plane = horizontal)
                rel = _marker_distance_2d(unplaced_tvec, placed_tvec)
                # Convert to cm
                rel_cm = rel * 100.0
                positions[unplaced_id] = placed_pos + rel_cm

    if len(positions) < 3:
        raise ValueError(
            f"Only {len(positions)} markers placed. Need at least 3 for a reliable transform. "
            "Make sure photos share markers to connect all groups."
        )

    return positions


# ── Homography computation ─────────────────────────────────────────────────────
def compute_transform(marker_positions_relative: dict, slam_positions: dict = None) -> dict:
    """
    Given relative marker positions (from photos) and known SLAM positions,
    compute homography H: SLAM(x,z) → room cm.
    
    If slam_positions not provided, uses SLAM_MARKER_POSITIONS fallback.
    Returns calibration dict.
    """
    slam_pos = slam_positions or load_slam_marker_positions() or SLAM_MARKER_POSITIONS

    # Find common markers
    common = set(marker_positions_relative.keys()) & set(slam_pos.keys())
    if len(common) < 3:
        raise ValueError(
            f"Only {len(common)} markers overlap between photos and SLAM data. Need 3+."
        )

    common = sorted(common)

    # Use photo-derived positions directly as room cm positions
    marker_positions_room_cm = {
        mid: list(marker_positions_relative[mid]) for mid in common
    }

    # Compute SLAM → room cm homography
    slam_src = np.float32([[slam_pos[mid] for mid in common]])
    room_dst = np.float32([[marker_positions_room_cm[mid] for mid in common]])

    if len(slam_src) < 4:
        # With < 4 points use affine
        H, _ = cv2.estimateAffinePartial2D(
            slam_src.reshape(-1, 1, 2),
            room_dst.reshape(-1, 1, 2)
        )
        # Pad to 3x3
        H = np.vstack([H, [0, 0, 1]])
        transform_source = "affine"
    else:
        H, _ = cv2.findHomography(
            slam_src.reshape(-1, 1, 2),
            room_dst.reshape(-1, 1, 2)
        )
        transform_source = "homography"

    # Validation: reprojection error
    validation_errors = []
    for mid in common:
        if mid not in marker_positions_room_cm:
            continue
        slam_pt = np.float32([[[slam_pos[mid][0], slam_pos[mid][1]]]])
        projected = cv2.perspectiveTransform(slam_pt, H)
        computed = [float(projected[0][0][0]), float(projected[0][0][1])]
        room = marker_positions_room_cm[mid]
        err = np.linalg.norm(np.array(computed) - np.array(room))
        validation_errors.append(err)

    validation_error_cm = float(np.mean(validation_errors)) if validation_errors else None

    # Also compute SLAM marker positions dict
    marker_positions_slam = {str(mid): list(slam_pos[mid]) for mid in common}
    marker_positions_real = {str(mid): marker_positions_room_cm[mid] for mid in marker_positions_room_cm}

    return {
        "transform_matrix": H.tolist(),
        "transform_source": transform_source,
        "marker_positions_real_cm": marker_positions_real,
        "marker_positions_slam": marker_positions_slam,
        "scale_cm_per_slam_unit": 244.43,
        "validation_error_cm": validation_error_cm,
        "source": "photo",
        "timestamp": datetime.now().isoformat(),
        "markers_used": common,
    }



# ── Photo-based wall-affine calibration ───────────────────────────────────────
def compute_transform_from_photos(photo_results: list[dict]) -> dict:
    """
    Compute SLAM→room transform from per-photo marker tvec data using
    wall-constrained optimization and full affine fitting.

    Room: 400 cm wide (X), 800 cm tall (Y). Origin top-left, X right, Y down.
    4 ArUco markers on walls:
      M0: top wall    → y=0,   x unknown
      M4: bottom wall → y=800, x unknown
      M7: left wall   → x=0,   y unknown
      M6: right wall  → x=400, y unknown

    Algorithm:
      1. Collect real-world distances between marker pairs from photo tvecs.
      2. Solve for unknown wall coordinates via Nelder-Mead minimisation.
      3. Compute full 2D affine (SLAM → room) via least-squares.
      4. Validate with reprojection error.

    Only uses markers {0, 4, 6, 7}.
    """
    from scipy.optimize import minimize

    TARGET_MARKERS = {0, 4, 6, 7}

    # ── Load SLAM positions ──────────────────────────────────────────────────
    slam_pos_all = load_slam_marker_positions() or SLAM_MARKER_POSITIONS
    slam_pos = {k: np.array([v[0], v[1]]) for k, v in slam_pos_all.items()
                if k in TARGET_MARKERS}

    if len(slam_pos) < 2:
        raise ValueError(
            f"Need SLAM positions for at least 2 of {TARGET_MARKERS}, "
            f"got {set(slam_pos.keys())}."
        )

    # ── Step 1: collect real-world distances from photos ─────────────────────
    # For each pair seen in a photo, accumulate distances for averaging
    pair_distances_accum = {}  # (min_id, max_id) → list of distances

    for markers in photo_results:
        detected = {
            k: v for k, v in markers.items()
            if k in TARGET_MARKERS and "tvec" in v
        }
        if len(detected) < 2:
            continue

        ids = sorted(detected.keys())
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                ma, mb = ids[i], ids[j]
                ta = np.array(detected[ma]["tvec"])
                tb = np.array(detected[mb]["tvec"])
                real_dist_cm = np.linalg.norm(ta - tb) * 100.0

                pair = (min(ma, mb), max(ma, mb))
                pair_distances_accum.setdefault(pair, []).append(real_dist_cm)

    # Average distances for pairs seen in multiple photos
    distances = {pair: float(np.mean(vals))
                 for pair, vals in pair_distances_accum.items()}

    if len(distances) < 2:
        raise ValueError(
            f"Only {len(distances)} distance pair(s) found from photos. "
            f"Need at least 2 pairs from markers {{0, 4, 6, 7}} to solve. "
            f"Make sure photos contain multiple markers from the target set."
        )

    # ── Step 2: solve for unknown wall positions via Nelder-Mead ─────────────
    # Variables: [x0, x4, y7, y6]
    # M0 → (x0,   0)
    # M4 → (x4, 800)
    # M7 → (0,   y7)
    # M6 → (400, y6)

    def build_room_pos(params):
        x0, x4, y7, y6 = params
        return {
            0: np.array([x0,   0.0]),
            4: np.array([x4, 800.0]),
            7: np.array([0.0,  y7]),
            6: np.array([400.0, y6]),
        }

    def loss_fn(params):
        x0, x4, y7, y6 = params
        room = build_room_pos(params)

        total = 0.0
        for (ma, mb), measured_cm in distances.items():
            if ma in room and mb in room:
                computed = np.linalg.norm(room[ma] - room[mb])
                total += (computed - measured_cm) ** 2

        # Soft bounds penalty
        penalty = 0.0
        for val, lo, hi in [(x0, 50, 350), (x4, 50, 350),
                            (y7, 100, 700), (y6, 100, 700)]:
            if val < lo:
                penalty += (lo - val) ** 2 * 100
            elif val > hi:
                penalty += (val - hi) ** 2 * 100

        return total + penalty

    x0_init = [200.0, 200.0, 450.0, 470.0]
    result = minimize(loss_fn, x0_init, method='Nelder-Mead',
                      options={'maxiter': 10000, 'xatol': 0.01, 'fatol': 0.01})

    if not result.success and result.fun > 1e6:
        raise ValueError(
            f"Nelder-Mead solver did not converge: {result.message}. "
            f"Final loss = {result.fun:.2f}"
        )

    solved_room = build_room_pos(result.x)

    # ── Step 3: compute full 2D affine transform SLAM → room ────────────────
    # room_X = a * slam_x + b * slam_z + c
    # room_Y = d * slam_x + e * slam_z + f
    # Set up as: A @ [a, b, c]^T = room_X_vec  (and same for Y)
    common = sorted(set(solved_room.keys()) & set(slam_pos.keys()))
    if len(common) < 3:
        raise ValueError(
            f"Only {len(common)} markers common between solved room and SLAM. "
            f"Need at least 3 for affine transform."
        )

    n = len(common)
    A = np.zeros((n, 3))
    bx = np.zeros(n)
    by = np.zeros(n)

    for i, mid in enumerate(common):
        sx, sz = slam_pos[mid][0], slam_pos[mid][1]
        A[i] = [sx, sz, 1.0]
        bx[i] = solved_room[mid][0]
        by[i] = solved_room[mid][1]

    # Solve via least squares
    res_x, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
    res_y, _, _, _ = np.linalg.lstsq(A, by, rcond=None)

    a, b, c = res_x
    d, e, f = res_y

    H = np.array([
        [a, b, c],
        [d, e, f],
        [0.0, 0.0, 1.0],
    ])

    # ── Step 4: validation ───────────────────────────────────────────────────
    errors = []
    for mid in common:
        slam_pt = np.array([slam_pos[mid][0], slam_pos[mid][1], 1.0])
        projected = H @ slam_pt
        err = np.linalg.norm(projected[:2] - solved_room[mid])
        errors.append(err)

    validation_error_cm = float(np.mean(errors))

    # ── Compute mean scale ───────────────────────────────────────────────────
    scale_samples = []
    for i in range(len(common)):
        for j in range(i + 1, len(common)):
            ma, mb = common[i], common[j]
            room_d = np.linalg.norm(solved_room[ma] - solved_room[mb])
            slam_d = np.linalg.norm(slam_pos[ma] - slam_pos[mb])
            if slam_d > 1e-6:
                scale_samples.append(room_d / slam_d)

    mean_scale = float(np.mean(scale_samples)) if scale_samples else 0.0

    # ── Assemble output ──────────────────────────────────────────────────────
    marker_positions_real_cm = {
        str(mid): [float(solved_room[mid][0]), float(solved_room[mid][1])]
        for mid in common
    }
    marker_positions_slam_out = {
        str(mid): [float(slam_pos[mid][0]), float(slam_pos[mid][1])]
        for mid in common
    }

    return {
        "transform_matrix": H.tolist(),
        "transform_source": "wall_affine",
        "marker_positions_real_cm": marker_positions_real_cm,
        "marker_positions_slam": marker_positions_slam_out,
        "scale_cm_per_slam_unit": mean_scale,
        "validation_error_cm": validation_error_cm,
        "source": "photo",
        "timestamp": datetime.now().isoformat(),
        "markers_used": [0, 4, 6, 7],
    }


# ── Apply transform ────────────────────────────────────────────────────────────
def slam_to_cm(slam_x: float, slam_z: float, H: np.ndarray) -> tuple[float, float]:
    """Convert SLAM (x, z) to room cm (X, Y) using homography H."""
    pt = np.float32([[[slam_x, slam_z]]])
    result = cv2.perspectiveTransform(pt, H)
    return float(result[0][0][0]), float(result[0][0][1])


