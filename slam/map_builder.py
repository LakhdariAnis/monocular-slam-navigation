#!/usr/bin/env python3
"""map_builder.py — Clean navigation map with auto-derived marker positions.

If MarkerPositions.txt is present (from slam_reader), marker positions are
computed from SLAM data via a least-squares SLAM→room transform.
Markers with too few observations fall back to tape-measure values.
The PNG visualization and stations.json always use the best available data.
"""

import argparse
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import binary_dilation

# ── Room geometry (cm) ────────────────────────────────────────────────────────
ROOM_W = 400.0   # X axis (left → right)
ROOM_H = 800.0   # Z axis (top → bottom)

# Tape-measure ground truth (cm) — wall-anchor values used for transform.
# Origin = top-left corner. X goes right, Z goes down.
# Room is 400 cm wide (X) × 800 cm tall (Y/Z).
TAPE_MEASURE = {
    0: (332.4,   0.0),   # top wall
    4: (125.2, 800.0),   # bottom wall
    7: (  0.0, 373.0),   # left wall
    6: (400.0, 523.0),   # right wall
}

# Which wall each marker belongs to
MARKER_WALL = {0: "top", 4: "bottom", 7: "left", 6: "right"}

# Minimum SLAM observations to trust a marker's SLAM position.
# Below this → fall back to tape-measure value.
MIN_OBS_THRESHOLD = 50

# Station approach: how far inside the wall (cm) the car actually navigates to
APPROACH_INSET_CM = 20.0

MARKER_COLORS = {7: "#1D9E75", 6: "#639922", 4: "#BA7517", 0: "#D85A30"}
MARKER_LABELS = {7: "M7", 6: "M6", 4: "M4", 0: "M0"}

# ── SLAM config (for .pgm costmap only) ──────────────────────────────────────
SCALE           = 96.23
Y_MIN           = -0.45
Y_MAX           = +0.35
FLOOR_Y_SLAM    = -0.07
OBSTACLE_MIN_CM =  5.0
OBSTACLE_MAX_CM = 85.0
OUTLIER_K       = 10
OUTLIER_THRESH  = 2.5
RESOLUTION_CM   = 2.0
INFLATE_CM      = 12.0
INFLATE_CELLS   = int(INFLATE_CM / RESOLUTION_CM)
MIN_PTS         = 2
MAX_DIST_CM     = 60.0
PGM_OCCUPIED    = 0
PGM_FREE        = 254


# ── Marker position loading ───────────────────────────────────────────────────

def load_marker_positions(path):
    """Read MarkerPositions.txt → dict: marker_id → (x, y, z, num_obs)"""
    markers = {}
    if not os.path.exists(path):
        print(f"[markers] {path} not found — will use tape-measure values for all markers.")
        return markers
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            mid  = int(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            obs  = int(parts[4]) if len(parts) == 5 else int(parts[8])
            markers[mid] = (x, y, z, obs)
    print(f"[markers] Loaded {len(markers)} markers from {path}")
    for mid, (x, y, z, obs) in markers.items():
        print(f"    M{mid}: SLAM ({x:.4f}, {y:.4f}, {z:.4f})  obs={obs}")
    return markers


# ── SLAM → room coordinate transform ─────────────────────────────────────────

def solve_slam_to_room(slam_markers):
    """
    Solve a 2D affine transform mapping SLAM coordinates → room cm.

    Monocular SLAM has anisotropic scale (the trajectory spans the room
    differently in X vs Z), so a similarity (uniform-scale) transform
    doesn't fit well. We use a full affine transform instead:

        room_X = a * slam_z + b * slam_x + c
        room_Z = d * slam_z + e * slam_x + f

    With 3 anchor markers this is solved exactly. With >3 it is
    over-determined and solved via least squares.

    Returns a transform function and the list of anchor marker IDs used.
    """
    # Collect anchors: markers with enough observations AND known tape pos
    rows  = []   # [slam_z, slam_x, 1]
    bx_   = []   # tape room X
    bz_   = []   # tape room Z
    used  = []

    for mid, (sx, sy, sz, obs) in slam_markers.items():
        if mid not in TAPE_MEASURE:
            continue
        if obs >= MIN_OBS_THRESHOLD:
            rows.append([sz, sx, 1.0])
            bx_.append(TAPE_MEASURE[mid][0])
            bz_.append(TAPE_MEASURE[mid][1])
            used.append(mid)

    print(f"[transform] Using {len(used)} anchor markers: {used}")

    if len(used) < 2:
        print("[transform] Not enough anchors (need ≥2) — falling back to tape-measure for all.")
        return None, used

    A  = np.array(rows,  dtype=float)
    bx = np.array(bx_,  dtype=float)
    bz = np.array(bz_,  dtype=float)

    if len(used) == 3:
        # Exactly determined — direct solve
        px = np.linalg.solve(A, bx)
        pz = np.linalg.solve(A, bz)
    else:
        # Over-determined — least squares
        px, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
        pz, _, _, _ = np.linalg.lstsq(A, bz, rcond=None)

    # Residuals
    pred_x = A @ px
    pred_z = A @ pz
    res = np.sqrt((pred_x - bx)**2 + (pred_z - bz)**2)
    for i, mid in enumerate(used):
        print(f"    M{mid}: residual {res[i]:.1f} cm")
    if len(used) > 2:
        print(f"    RMS residual: {np.sqrt((res**2).mean()):.1f} cm")

    def transform(slam_x, slam_z):
        """Convert SLAM (x, z) → room (X_cm, Z_cm)"""
        row = np.array([slam_z, slam_x, 1.0])
        return float(row @ px), float(row @ pz)

    return transform, used


# ── Wall snapping ─────────────────────────────────────────────────────────────

def snap_to_wall(room_x, room_z, wall):
    """Project a point onto its assigned wall and clamp to room bounds."""
    margin = 5.0  # don't snap closer than 5cm to a corner
    if wall == "top":
        return (np.clip(room_x, margin, ROOM_W - margin), 0.0)
    elif wall == "bottom":
        return (np.clip(room_x, margin, ROOM_W - margin), ROOM_H)
    elif wall == "left":
        return (0.0, np.clip(room_z, margin, ROOM_H - margin))
    elif wall == "right":
        return (ROOM_W, np.clip(room_z, margin, ROOM_H - margin))
    return (room_x, room_z)


def approach_point(marker_x, marker_z, wall):
    """20 cm inset from wall toward room center."""
    d = APPROACH_INSET_CM
    if wall == "top":    return (marker_x, d)
    if wall == "bottom": return (marker_x, ROOM_H - d)
    if wall == "left":   return (d, marker_z)
    if wall == "right":  return (ROOM_W - d, marker_z)
    return (marker_x, marker_z)


# ── Build final MARKERS dict ──────────────────────────────────────────────────

def build_marker_positions(slam_markers):
    """
    Returns MARKERS dict: mid → (room_X_cm, room_Z_cm)
    Uses SLAM-derived positions where reliable, tape-measure otherwise.
    """
    transform, anchors = solve_slam_to_room(slam_markers)

    markers = {}
    sources = {}

    for mid in TAPE_MEASURE:
        wall = MARKER_WALL[mid]

        use_slam = (
            transform is not None
            and mid in slam_markers
            and slam_markers[mid][3] >= MIN_OBS_THRESHOLD
        )

        if use_slam:
            sx, sy, sz, obs = slam_markers[mid]
            raw_x, raw_z = transform(sx, sz)
            snapped = snap_to_wall(raw_x, raw_z, wall)
            markers[mid] = snapped
            sources[mid] = f"SLAM ({obs} obs) → raw ({raw_x:.1f}, {raw_z:.1f}) → snapped {snapped}"
        else:
            markers[mid] = TAPE_MEASURE[mid]
            obs = slam_markers[mid][3] if mid in slam_markers else 0
            reason = f"obs={obs} < {MIN_OBS_THRESHOLD}" if obs else "not in SLAM data"
            sources[mid] = f"TAPE MEASURE (fallback: {reason})"

    print("\n[markers] Final marker positions:")
    for mid in sorted(markers):
        mx, mz = markers[mid]
        print(f"    M{mid} ({MARKER_WALL[mid]:6s} wall): ({mx:.1f}, {mz:.1f}) cm  ← {sources[mid]}")

    return markers


# ── SLAM point cloud helpers ──────────────────────────────────────────────────

def load_map_points(path):
    print(f"[1] Loading map points from {path} ...")
    pts = np.loadtxt(path)
    print(f"    {len(pts)} raw points")
    return pts

def load_trajectory(path):
    print(f"[1] Loading trajectory from {path} ...")
    data = np.loadtxt(path)
    return data[:, 1:4]

def filter_y(pts):
    mask = (pts[:,1] >= Y_MIN) & (pts[:,1] <= Y_MAX)
    r = pts[mask]
    print(f"[2] Y filter: {len(pts)} → {len(r)}")
    return r

def remove_outliers(pts):
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    dists, _ = tree.query(pts, k=OUTLIER_K+1)
    md = dists[:,1:].mean(axis=1)
    mask = md < (md.mean() + OUTLIER_THRESH * md.std())
    r = pts[mask]
    print(f"[3] Outlier removal: {len(pts)} → {len(r)}")
    return r

def filter_obstacle_band(pts):
    obs_min = OBSTACLE_MIN_CM / SCALE
    obs_max = OBSTACLE_MAX_CM / SCALE
    y_low   = FLOOR_Y_SLAM + obs_min
    y_high  = FLOOR_Y_SLAM + obs_max
    mask = (pts[:,1] >= y_low) & (pts[:,1] <= y_high)
    r = pts[mask]
    print(f"[4] Obstacle band: {len(pts)} → {len(r)}")
    return r if len(r) >= 50 else pts

def filter_path_proximity(pts, traj):
    from scipy.spatial import cKDTree
    traj_xz = np.column_stack([traj[:,2], traj[:,0]]) * SCALE
    pts_xz  = np.column_stack([pts[:,2],  pts[:,0]])  * SCALE
    tree = cKDTree(traj_xz)
    dists, _ = tree.query(pts_xz, k=1)
    r = pts[dists <= MAX_DIST_CM]
    print(f"[5] Path proximity: {len(pts)} → {len(r)}")
    return r

def to_cm(pts):
    return np.column_stack([pts[:,2] * SCALE, pts[:,0] * SCALE])

def traj_to_cm(traj):
    return np.column_stack([traj[:,2] * SCALE, traj[:,0] * SCALE])


# ── PGM costmap ───────────────────────────────────────────────────────────────

def build_grid(pts_cm):
    pad  = 20.0
    x_min = -pad;  x_max = ROOM_W + pad
    z_min = -pad;  z_max = ROOM_H + pad
    cols = int(np.ceil((x_max - x_min) / RESOLUTION_CM))
    rows = int(np.ceil((z_max - z_min) / RESOLUTION_CM))
    count = np.zeros((rows, cols), dtype=np.int32)
    xi = np.clip(((pts_cm[:,0] - x_min) / RESOLUTION_CM).astype(int), 0, cols-1)
    zi = np.clip(((pts_cm[:,1] - z_min) / RESOLUTION_CM).astype(int), 0, rows-1)
    for gx, gz in zip(xi, zi):
        count[gz, gx] += 1
    occupied = count >= MIN_PTS
    struct   = np.ones((INFLATE_CELLS*2+1, INFLATE_CELLS*2+1), bool)
    inflated = binary_dilation(occupied, structure=struct)

    def wall_cells(x0, z0, x1, z1, thickness=2):
        for dx in range(thickness):
            for dz in range(thickness):
                gx0 = int((x0 - x_min) / RESOLUTION_CM) + dx
                gz0 = int((z0 - z_min) / RESOLUTION_CM) + dz
                gx1 = int((x1 - x_min) / RESOLUTION_CM) + dx
                gz1 = int((z1 - z_min) / RESOLUTION_CM) + dz
                for gx in range(min(gx0,gx1), max(gx0,gx1)+1):
                    for gz in range(min(gz0,gz1), max(gz0,gz1)+1):
                        if 0 <= gx < cols and 0 <= gz < rows:
                            inflated[gz, gx] = True

    wall_cells(0, 0, ROOM_W, 0)
    wall_cells(0, ROOM_H, ROOM_W, ROOM_H)
    wall_cells(0, 0, 0, ROOM_H)
    wall_cells(ROOM_W, 0, ROOM_W, ROOM_H)
    print(f"[6] Grid: {cols}×{rows}px  occupied={occupied.sum()}  inflated={inflated.sum()}")
    return inflated, (x_min, z_min)

def save_pgm(occupied, path):
    rows, cols = occupied.shape
    grid = np.full((rows, cols), PGM_FREE, dtype=np.uint8)
    grid[occupied] = PGM_OCCUPIED
    grid = np.flipud(grid)
    with open(path, "wb") as f:
        f.write(f"P5\n{cols} {rows}\n255\n".encode())
        f.write(grid.tobytes())
    print(f"[7] Saved {path}")

def save_yaml(path, pgm_path, origin_cm):
    ox, oy = origin_cm[0]/100.0, origin_cm[1]/100.0
    with open(path, "w") as f:
        f.write(f"""image: {os.path.basename(pgm_path)}
resolution: {RESOLUTION_CM/100.0:.4f}
origin: [{ox:.4f}, {oy:.4f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")


# ── PNG visualization ─────────────────────────────────────────────────────────

def save_visualization(png_path, markers, stations, slam_source):
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.set_facecolor("#1a1a2e")
    fig.patch.set_facecolor("#1a1a2e")

    room_rect = patches.Rectangle(
        (0, 0), ROOM_W, ROOM_H,
        linewidth=3, edgecolor="#ffffff", facecolor="#ffffff08",
        zorder=2
    )
    ax.add_patch(room_rect)

    for mid, (mx, mz) in markers.items():
        color = MARKER_COLORS[mid]
        label = MARKER_LABELS[mid]
        sx, sz = stations[mid]

        ax.scatter(mx, mz, s=300, c=color, edgecolors="white",
                   marker="s", zorder=5, linewidths=1.5)
        ax.scatter(sx, sz, s=120, facecolors="none", edgecolors=color,
                   marker="o", zorder=4, linewidths=1.5, linestyles="--")
        ax.plot([mx, sx], [mz, sz], color=color, linewidth=0.8,
                linestyle=":", alpha=0.6, zorder=3)

        offsets = {7: (0, 8), 6: (0, -10), 4: (8, 0), 0: (-8, 0)}
        ox, oz = offsets[mid]
        ha = "left" if ox >= 0 else "right"
        va = "bottom" if oz >= 0 else "top"
        src_tag = "" if slam_source.get(mid, False) else " [tape]"
        ax.annotate(
            f"{label}{src_tag}\n({mx:.0f}, {mz:.0f})",
            (mx, mz),
            xytext=(mx + ox, mz + oz),
            fontsize=10, fontweight="bold", color=color,
            ha=ha, va=va,
            bbox=dict(boxstyle="round,pad=0.3", fc="#000000bb", ec=color, lw=0.8),
            zorder=6
        )

    ax.annotate("", xy=(ROOM_W, -12), xytext=(0, -12),
                arrowprops=dict(arrowstyle="<->", color="#aaaaaa", lw=1))
    ax.text(ROOM_W/2, -18, f"{ROOM_W:.0f} cm", color="#aaaaaa",
            ha="center", va="top", fontsize=9)
    ax.annotate("", xy=(-18, ROOM_H), xytext=(-18, 0),
                arrowprops=dict(arrowstyle="<->", color="#aaaaaa", lw=1))
    ax.text(-24, ROOM_H/2, f"{ROOM_H:.0f} cm", color="#aaaaaa",
            ha="right", va="center", fontsize=9, rotation=90)

    ax.scatter([], [], s=80, facecolors="none", edgecolors="#aaaaaa",
               marker="o", linestyles="--", label="Station approach point")
    ax.scatter([], [], s=200, c="#aaaaaa", edgecolors="white",
               marker="s", label="ArUco marker (wall)")

    ax.set_xlim(-40, ROOM_W + 40)
    ax.set_ylim(-35, ROOM_H + 35)
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.set_xlabel("X (cm)", color="white", fontsize=11)
    ax.set_ylabel("Z (cm)  [top=0, bottom=146]", color="white", fontsize=11)
    ax.tick_params(colors="white")
    ax.set_title(f"Navigation Map — Room {ROOM_W:.0f}×{ROOM_H:.0f} cm", color="white", fontsize=13)
    ax.legend(loc="upper right", facecolor="#333333", labelcolor="white", fontsize=9)
    ax.grid(True, alpha=0.15, color="white")
    plt.tight_layout()
    plt.savefig(png_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"[8] Saved {png_path}")


# ── stations.json ─────────────────────────────────────────────────────────────

def save_stations(path, markers, stations, slam_source):
    out = {}
    for mid, (mx, mz) in markers.items():
        sx, sz = stations[mid]
        out[f"marker_{mid}"] = {
            "marker_x_cm":    round(mx, 1),
            "marker_z_cm":    round(mz, 1),
            "approach_x_cm":  round(sx, 1),
            "approach_z_cm":  round(sz, 1),
            "wall":           MARKER_WALL[mid],
            "source":         "slam" if slam_source.get(mid) else "tape_measure",
        }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[9] Saved {path}")
    for k, v in out.items():
        print(f"    {k}: approach ({v['approach_x_cm']}, {v['approach_z_cm']}) cm  [{v['source']}]")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--points",   default="MapPoints.txt")
    parser.add_argument("--traj",     default="KeyFrameTrajectory.txt")
    parser.add_argument("--markers",  default="MarkerPositions.txt",
                        help="Path to MarkerPositions.txt from slam_reader")
    parser.add_argument("--out",      default=".")
    parser.add_argument("--no-slam",  action="store_true",
                        help="Skip SLAM point cloud processing (PNG + stations only)")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    pgm_path      = os.path.join(args.out, "map.pgm")
    yaml_path     = os.path.join(args.out, "map.yaml")
    png_path      = os.path.join(args.out, "map_final.png")
    stations_path = os.path.join(args.out, "stations.json")

    # ── Marker positions (Step A) ──────────────────────────────────────────
    slam_markers = load_marker_positions(args.markers)
    markers      = build_marker_positions(slam_markers)

    # Track which markers came from SLAM vs tape
    slam_source = {
        mid: (mid in slam_markers and slam_markers[mid][3] >= MIN_OBS_THRESHOLD)
        for mid in TAPE_MEASURE
    }

    # ── Station approach points ────────────────────────────────────────────
    stations = {
        mid: approach_point(mx, mz, MARKER_WALL[mid])
        for mid, (mx, mz) in markers.items()
    }

    # ── SLAM point cloud → .pgm costmap ───────────────────────────────────
    if not args.no_slam:
        pts  = load_map_points(args.points)
        traj = load_trajectory(args.traj)
        pts  = filter_y(pts)
        pts  = remove_outliers(pts)
        pts  = filter_obstacle_band(pts)
        pts  = filter_path_proximity(pts, traj)
        pts_cm = to_cm(pts)
        occupied, origin_cm = build_grid(pts_cm)
        save_pgm(occupied, pgm_path)
        save_yaml(yaml_path, pgm_path, origin_cm)
    else:
        print("[SLAM skipped] PNG and stations only.")

    save_visualization(png_path, markers, stations, slam_source)
    save_stations(stations_path, markers, stations, slam_source)

    print()
    print("Done.")
    print(f"  Room:     {ROOM_W} × {ROOM_H} cm")
    print(f"  Markers:  {list(markers.keys())}")
    print(f"  PNG:      {png_path}")
    print(f"  Stations: {stations_path}")


if __name__ == "__main__":
    main()
