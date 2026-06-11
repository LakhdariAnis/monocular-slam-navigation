# SLAM Layer

This directory contains a custom build of ORB-SLAM3 with two key additions: ArUco-based absolute relocalization and a ZMQ pose publisher. The system runs in **monocular-only** mode — no depth sensor is available. Because a Raspberry Pi 3B (1 GB RAM) cannot sustain real-time ORB feature extraction, bundle adjustment, and loop closure simultaneously, SLAM runs on a laptop/PC. The Pi streams compressed JPEG frames over ZMQ to the PC; the PC runs `slam_reader`, tracks the camera pose, and publishes poses back over a second ZMQ channel that the navigator subscribes to.

---

## Why Custom ORB-SLAM3

Upstream ORB-SLAM3 had two problems that made it unusable for a mobile robot in this environment:

**1. No external pose injection API.**
When visual tracking is lost, upstream ORB-SLAM3 relies solely on BoW-based place recognition to relocalize. In a feature-poor indoor environment (plain painted walls), this can take several seconds or fail indefinitely. During that window the car drives blind with no position estimate. The fix required adding `InjectPoseHint(Tcw)` to `System.h`/`System.cc` so that an external absolute fix — computed in one frame from an ArUco marker — can be injected directly into the tracker's motion model before the next `TrackMonocular` call.

**2. Atlas serialization crashes.**
Saving the map on exit (required to reuse it across sessions) caused segfaults. The root cause was iterator invalidation: `Map::PreSave` iterated over `mspMapPoints` using a set iterator while `EraseObservation → SetBadFlag → EraseMapPoint` could erase elements from that same set mid-loop. The fix snapshots the set into a `std::vector` before iterating. See `CHANGES.md` for full details on all three crash-related fixes.

---

## Architecture

```
[Pi Cam v2] ──ZMQ:5555──→ [bridge.py] ──POSIX shm──→ [slam_reader C++]
                                                              │
                                                       ZMQ:5557 (JSON poses)
                                                              ↓
                                                       [slam_zmq.py]
                                                              ↓
                                                       [navigator.py]
```

### Component Descriptions

**`bridge.py`** — Subscribes to the Pi's ZMQ PUB on port 5555, receives JPEG frames, decodes them to BGR with OpenCV, and writes each frame plus a sequence number and timestamp into POSIX shared memory (`/orbframe`, 921 616 bytes). Decoupling network I/O from the C++ SLAM thread prevents a slow network from stalling the tracker.

**`slam_reader` (C++)** — Main SLAM binary. Reads frames from shared memory using a double-read consistency check on the sequence number, optionally detects ArUco markers and stamps their corners as ORB features, feeds each frame to `ORB_SLAM3::System::TrackMonocular`, and publishes the resulting SE3 pose as JSON on ZMQ port 5557 after every unique frame. In mapping mode it also collects per-frame ArUco SE3 observations for later averaging.

**`slam_zmq.py`** — Python supervisor: launches `slam_reader` as a subprocess in localization mode, parses its stdout for tracked-pose lines, tracking-lost lines, and the ORB-SLAM3 internal "Reseting active map" string, then relays enriched JSON (with a `reset` flag) on port 5557. It also opens a REP socket on port 5558 that the navigator use to send an `"ACK"` string once it has recovered from a map reset, clearing the reset latch.

---

## Operating Modes

### Mapping mode (`--map`)

```bash
./slam_reader ORBvoc.txt picam.yaml --map [--no-viewer]
```

Builds a fresh map from the live camera feed. Walk the car around the room and ensure all ArUco markers are observed from multiple angles. On `Ctrl+C`, saves:

| File | Contents |
|------|----------|
| `MapPoints.txt` | 3D point cloud — one `x y z` per line (SLAM units) |
| `MarkerPositions.txt` | ArUco SE3 poses — `id tx ty tz qw qx qy qz obs` per line |
| `KeyFrameTrajectory.txt` | TUM-format keyframe trajectory |
| `room_map.osa` | Full binary ORB-SLAM3 atlas (Boost serialization) |

After saving, `slam_reader` automatically invokes `map_builder.py` to compute the SLAM→room transform and write `data/stations.json`.

### Localization mode (`--loc <atlas>`)

```bash
./slam_reader ORBvoc.txt picam.yaml --loc room_map [--no-viewer]
```

Loads the saved atlas, activates localization mode (`SLAM.ActivateLocalizationMode()`), and publishes live poses on ZMQ port 5557 at up to ~30 fps. The map is never modified. If tracking is lost and a known ArUco marker is visible, the ArUco relocalization state machine fires (see below).

---

## ArUco Relocalization

**Problem:** Standard ORB visual relocalization is slow and unreliable when walls are plain (low texture → sparse BoW matches → no relocalization candidates).

**Solution:** ArUco markers (DICT_4X4_50, 18.4 cm physical size) are printed and taped to the four walls. Their 3D poses in SLAM coordinates are computed during mapping and saved to `MarkerPositions.txt`.

**Relocalization procedure (localization mode):**

1. `cv::aruco::detectMarkers` scans each incoming frame.
2. On the first lost frame where a known marker is visible, `SLAM.DeactivateLocalizationMode()` is called — this lets ORB-SLAM3 add new keyframes and rebuild local map context (full SLAM mode), which is required for the tracker to have enough structure to re-establish tracking.
3. Simultaneously, `cv::aruco::estimatePoseSingleMarkers` runs PnP on the marker corners to get the marker-in-camera transform `T_cm`. Combined with the stored marker-in-world transform `T_wm` (loaded from `MarkerPositions.txt`), the camera-in-world pose is recovered:

   ```
   T_wc = T_wm * T_cm⁻¹
   Tcw  = T_wc⁻¹        (ORB-SLAM3 convention)
   ```

4. `SLAM.InjectPoseHint(Tcw)` writes `Tcw` into `mpTracker->mLastFrame`, zeros the velocity model, and sets tracking state to `OK`, so the next `TrackMonocular` call starts `TrackWithMotionModel` from the ArUco-derived position instead of from a stale or zero pose.
5. Tracking restarts on the next frame. `ActivateLocalizationMode()` is re-engaged once `GetTrackingState()` returns 2 (`Tracking::OK`).
6. If recovery does not occur within `RELOC_MAX_FRAMES` = 60 frames (~2 s at 30 fps), the state machine resets and immediately retries on the next lost frame that has a visible marker.

**Corner stamping:** To make ArUco corners appear as strong, repeatable ORB features, white squares of radius `ARUCO_CORNER_RADIUS` = 6 px are painted over every corner before ORB extraction. This radius **must be identical** between mapping and localization sessions; a different radius produces different descriptors and breaks BoW map matching.

**Marker images:** `slam/marker_0.png` through `slam/marker_7.png` — print at 18.4 cm and laminate.

---

## ZMQ Pose Format

**Port:** 5557 (PUB, `ZMQ_NOBLOCK`). Published once per unique frame from shared memory, scaling with camera ingestion rate (~30 fps).

```json
{"seq":1234,"ts":1720000000.123456,"ok":true,"x":-0.045,"y":-0.081,"z":0.430,"R":[r00,r01,r02,r10,r11,r12,r20,r21,r22]}
{"seq":1235,"ts":1720000000.157,"ok":false,"x":null,"y":null,"z":null,"R":null}
```

*Exact serializer from `slam_reader.cc` (lines 117–136):*

```cpp
ss << "{\"seq\":" << seq << ",\"ts\":" << ts
   << ",\"ok\":" << (ok ? "true" : "false");
if (ok) {
    Eigen::Vector3f t = cam_position(Tcw);   // Tcw.inverse().translation()
    Eigen::Matrix3f R = Tcw.inverse().rotationMatrix();
    ss << ",\"x\":" << t.x() << ",\"y\":" << t.y() << ",\"z\":" << t.z();
    ss << ",\"R\":[";
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            ss << R(r, c);
            if (r * 3 + c < 8) ss << ",";
        }
    ss << "]";
} else {
    ss << ",\"x\":null,\"y\":null,\"z\":null,\"R\":null";
}
ss << "}";
```

| Field | Type | Description |
|-------|------|-------------|
| `seq` | uint64 | Frame sequence counter from shared memory header |
| `ts` | float64 | Unix timestamp in seconds (from shared memory header) |
| `ok` | bool | `true` if `GetTrackingState() == 2` (Tracking::OK) |
| `x`, `y`, `z` | float64 / null | Camera position in SLAM metres — `Tcw.inverse().translation()` |
| `R` | float64[9] / null | 3×3 rotation matrix, **row-major**, from `Tcw.inverse().rotationMatrix()` |

Note: `x/y/z/R` are `null` whenever `ok` is `false`.

`slam_zmq.py` adds a `reset` boolean field to indicate a map-frame discontinuity:

```json
{"seq":-1,"ts":0.0,"ok":false,"x":null,"y":null,"z":null,"reset":true}
```

The navigator must not use poses that carry `reset: true` for navigation until it sends `"ACK"` on port 5558 and receives `"OK"`.

---

## Build

```bash
cd slam/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DWITH_ARUCO=ON \
         -DWITH_ZMQ=ON \
         -DWITH_POSE_INJECT=ON
make -j4
```

### Dependencies

| Dependency | Notes |
|---|---|
| ORB-SLAM3 | Git submodule at `ORB_SLAM3/`; must be compiled first |
| OpenCV ≥ 4.x with `opencv_aruco` | Required for marker detection |
| Eigen3 | Header-only; usually pre-installed |
| libzmq + czmq | `sudo apt install libzmq3-dev` |
| Sophus | Bundled with ORB-SLAM3 thirdparty |
| Boost serialization | Required by ORB-SLAM3 atlas save/load |

Build ORB-SLAM3 first:

```bash
cd ORB_SLAM3
chmod +x build.sh && ./build.sh
```

Then build `slam_reader`:

```bash
cd slam/
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DWITH_ARUCO=ON -DWITH_ZMQ=ON -DWITH_POSE_INJECT=ON
make -j4
```

---

## Map Builder

`map_builder.py` is a post-processing script invoked automatically at the end of every mapping session. It accepts:

```
--points   MapPoints.txt
--traj     KeyFrameTrajectory.txt
--markers  MarkerPositions.txt
--out      <output directory>
```

**Processing pipeline:**

1. Loads `MarkerPositions.txt` (SE3 per marker, averaged over all observations where `obs ≥ 50`).
2. Solves a **2D affine transform** from SLAM coordinates to room centimetres using markers with known tape-measure ground truth (markers 0, 4, 6, 7) as anchors. Monocular SLAM has anisotropic scale (X and Z spans differ), so a full affine rather than a similarity transform is used.
3. Snaps each transformed marker position to its assigned wall and computes a 20 cm inset approach point.
4. Filters the SLAM point cloud (Y-band, outlier removal, obstacle-height band, path proximity) and rasterizes it into a `.pgm` occupancy grid costmap.
5. Writes:
   - `data/stations.json` — approach point per marker in room cm
   - `data/map_final.png` — visualization with room outline, markers, and approach circles
   - `data/map.pgm` / `data/map.yaml` — ROS-compatible occupancy costmap

---

## Custom Modifications

Full technical details for each change are in `CHANGES.md`. Summary:

| Change | File(s) | What it does |
|--------|---------|--------------|
| `InjectPoseHint` API | `System.h`, `System.cc` | Allows external code to seed the tracker with a known SE3 pose when lost, enabling single-frame ArUco recovery |
| `ForceRelocalization` helper | `System.cc` | Companion to pose injection — forces tracker state to `RECENTLY_LOST` so relocalization is attempted immediately |
| Atlas serialization crash fix | `Map.cc` (`PreSave`, `PostLoad`) | Snapshots `mspMapPoints` / `mspKeyFrames` into `std::vector` before iterating to prevent iterator invalidation from concurrent `EraseMapPoint` calls |
| Cross-map relocalization fix | `KeyFrameDatabase.cc` (`DetectRelocalizationCandidates`) | Removes the `pKFi->GetMap() == pMap` guard so keyframes from the stored (non-active) atlas map are included in relocalization candidates |
| Shutdown race condition fix | `System.cc` (`Shutdown`) | Waits for `LocalMapper` and `LoopCloser` to fully finish before calling `SaveAtlas`, preventing a data race between the serialization and map-mutation threads |
