"""
map_viewer.py — visualize SLAM map points as occupancy grid
Usage: python3 map_viewer.py

Reads:  ~/autonomous_car/data/MapPoints.txt
        ~/autonomous_car/navigation/stations.json   (optional)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import json, os

# ── PATHS — edit if yours differ ─────────────────────────────────────────────
MAPPOINTS_FILE = os.path.expanduser("~/autonomous_car/data/MapPoints.txt")
STATIONS_FILE  = os.path.expanduser("~/autonomous_car/navigation/stations.json")

# ── GRID SETTINGS ─────────────────────────────────────────────────────────────
CELL_SIZE   = 0.03   # meters per cell (3cm) — lower = finer grid
CAR_RADIUS  = 0.15   # inflation radius around obstacles (car half-width ~15cm)

# ── STATION COLORS ────────────────────────────────────────────────────────────
STATION_STYLE = {
    "start":     {"color": "#aaaaaa", "marker": "s", "size": 120},
    "station_1": {"color": "#00ff88", "marker": "o", "size": 140},
    "station_2": {"color": "#ffaa00", "marker": "o", "size": 140},
    "station_3": {"color": "#3388ff", "marker": "o", "size": 140},
    "station_4": {"color": "#ff44aa", "marker": "o", "size": 140},
}

# ── LOAD MAP POINTS ───────────────────────────────────────────────────────────
print(f"Loading map points from {MAPPOINTS_FILE}...")
pts = np.loadtxt(MAPPOINTS_FILE)   # shape (N, 3): x y z
print(f"  {len(pts)} points loaded")

# use x and z only (y is height — ignore it)
px_raw = pts[:, 0]
pz_raw = pts[:, 2]

# filter to dense cluster only — removes outliers beyond real room
X_MIN, X_MAX = -0.60, 0.65
Z_MIN, Z_MAX = -0.50, 1.55
mask = (px_raw >= X_MIN) & (px_raw <= X_MAX) & (pz_raw >= Z_MIN) & (pz_raw <= Z_MAX)
px = px_raw[mask]
pz = pz_raw[mask]
print(f"  Kept {mask.sum()} / {len(pts)} points after outlier filter")

# ── BUILD OCCUPANCY GRID ──────────────────────────────────────────────────────
# add padding around the point cloud
PAD = 0.3  # meters padding around bounding box

x_min, x_max = px.min() - PAD, px.max() + PAD
z_min, z_max = pz.min() - PAD, pz.max() + PAD

cols = int((x_max - x_min) / CELL_SIZE) + 1
rows = int((z_max - z_min) / CELL_SIZE) + 1

print(f"  Grid: {cols} x {rows} cells at {CELL_SIZE*100:.0f}cm resolution")
print(f"  X range: {x_min:.3f} → {x_max:.3f}")
print(f"  Z range: {z_min:.3f} → {z_max:.3f}")

# raw obstacle grid
grid = np.zeros((rows, cols), dtype=np.float32)

def world_to_cell(x, z):
    col = int((x - x_min) / CELL_SIZE)
    row = int((z - z_min) / CELL_SIZE)
    return row, col

def cell_to_world(row, col):
    x = x_min + col * CELL_SIZE + CELL_SIZE / 2
    z = z_min + row * CELL_SIZE + CELL_SIZE / 2
    return x, z

# mark raw obstacle cells
for x, z in zip(px, pz):
    r, c = world_to_cell(x, z)
    if 0 <= r < rows and 0 <= c < cols:
        grid[r, c] += 1

# normalize: any cell with points = obstacle
raw_obstacle = grid > 0

# inflate obstacles by car radius
inflate_cells = int(CAR_RADIUS / CELL_SIZE) + 1
from scipy.ndimage import binary_dilation
struct = np.ones((inflate_cells*2+1, inflate_cells*2+1), dtype=bool)
inflated = binary_dilation(raw_obstacle, structure=struct)

# build display grid: 0=free, 0.5=inflated, 1=obstacle
display = np.zeros((rows, cols), dtype=np.float32)
display[inflated]     = 0.4
display[raw_obstacle] = 1.0

print(f"  Raw obstacle cells:    {raw_obstacle.sum()}")
print(f"  Inflated (no-go zone): {inflated.sum()}")
print(f"  Free cells:            {(~inflated).sum()}")

# ── LOAD STATIONS ─────────────────────────────────────────────────────────────
stations = {}
if os.path.exists(STATIONS_FILE):
    with open(STATIONS_FILE) as f:
        stations = json.load(f)
    print(f"\nStations loaded from {STATIONS_FILE}")
else:
    print(f"\nNo stations file found at {STATIONS_FILE}")

# ── PLOT ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 8))
fig.patch.set_facecolor('#0a0a0f')

for ax in axes:
    ax.set_facecolor('#0a0a0f')
    ax.tick_params(colors='#555570')
    ax.spines[:].set_color('#1e1e2e')
    for spine in ax.spines.values():
        spine.set_edgecolor('#1e1e2e')

# ── LEFT: raw map points scatter ──────────────────────────────────────────────
ax1 = axes[0]
ax1.scatter(px, pz, s=2, c='#00ff88', alpha=0.4, linewidths=0)
ax1.set_title('Raw SLAM Map Points (x/z only)', color='#e0e0e0', fontsize=12, pad=10)
ax1.set_xlabel('X (meters)', color='#555570')
ax1.set_ylabel('Z (meters)', color='#555570')
ax1.set_aspect('equal')
ax1.grid(True, color='#1e1e2e', linewidth=0.5)
ax1.tick_params(colors='#555570')

# mark stations on scatter
for name, st in stations.items():
    sx, sz = st['x'], st['z']
    if sx == 0 and sz == 0 and name not in ('start',):
        continue
    style = STATION_STYLE.get(name, {"color":"white","marker":"o","size":100})
    ax1.scatter(sx, sz, c=style['color'], marker=style['marker'],
                s=style['size'], zorder=5, edgecolors='white', linewidths=0.8)
    ax1.annotate(name.replace('_',' ').title(),
                 (sx, sz), textcoords='offset points', xytext=(6, 4),
                 color=style['color'], fontsize=8, fontweight='bold')

# ── RIGHT: occupancy grid ─────────────────────────────────────────────────────
ax2 = axes[1]

# custom colormap: black=free, dark red=inflated, bright=obstacle
from matplotlib.colors import ListedColormap
cmap = ListedColormap(['#0d1117', '#2d1020', '#ff3366'])

# extent: [x_min, x_max, z_min, z_max]
extent = [x_min, x_max, z_min, z_max]
ax2.imshow(display, origin='lower', extent=extent,
           cmap=cmap, vmin=0, vmax=1, aspect='equal', interpolation='nearest')

ax2.set_title('Occupancy Grid (inflated for car size)', color='#e0e0e0', fontsize=12, pad=10)
ax2.set_xlabel('X (meters)', color='#555570')
ax2.set_ylabel('Z (meters)', color='#555570')
ax2.grid(True, color='#1e1e2e', linewidth=0.5, alpha=0.5)
ax2.tick_params(colors='#555570')

# mark stations on grid
for name, st in stations.items():
    sx, sz = st['x'], st['z']
    if sx == 0 and sz == 0 and name not in ('start',):
        continue
    style = STATION_STYLE.get(name, {"color":"white","marker":"o","size":100})
    ax2.scatter(sx, sz, c=style['color'], marker=style['marker'],
                s=style['size'], zorder=5, edgecolors='white', linewidths=0.8)
    ax2.annotate(name.replace('_',' ').title(),
                 (sx, sz), textcoords='offset points', xytext=(6, 4),
                 color=style['color'], fontsize=8, fontweight='bold')

    # check if station is in obstacle or inflated zone
    r, c = world_to_cell(sx, sz)
    if 0 <= r < rows and 0 <= c < cols:
        if raw_obstacle[r, c]:
            ax2.annotate('⚠ IN OBSTACLE', (sx, sz),
                         textcoords='offset points', xytext=(6, -14),
                         color='#ff3366', fontsize=7)
        elif inflated[r, c]:
            ax2.annotate('⚠ IN NO-GO ZONE', (sx, sz),
                         textcoords='offset points', xytext=(6, -14),
                         color='#ffaa00', fontsize=7)

# legend
legend_items = [
    mpatches.Patch(color='#0d1117', label='Free space'),
    mpatches.Patch(color='#2d1020', label=f'No-go zone (car radius {CAR_RADIUS*100:.0f}cm)'),
    mpatches.Patch(color='#ff3366', label='Obstacle (SLAM points)'),
]
ax2.legend(handles=legend_items, loc='upper right',
           facecolor='#111118', edgecolor='#1e1e2e',
           labelcolor='#e0e0e0', fontsize=8)

plt.tight_layout(pad=2)
plt.suptitle('SLAM Map Viewer', color='#e0e0e0', fontsize=14,
             fontweight='bold', y=1.01)

print("\nShowing map — close window to exit")
plt.show()
