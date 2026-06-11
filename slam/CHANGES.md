# ORB-SLAM3 Custom Modifications

This directory contains a modified version of ORB-SLAM3. This document describes
every change made to the upstream codebase and why.

## Overview
This modified ORB-SLAM3 system functions as a dual-mode monocular tracker with an enhanced ability to read live camera frames from shared memory, stream exact camera poses over ZMQ, and perform rapid, absolute relocalization using ArUco markers. Unlike the upstream version, it provides deterministic pose injection to force the ORB tracking motion model to instantly restart from a known global pose whenever tracking is lost.

## New Features

### 1. ArUco Marker Relocalization
It detects ArUco markers (`cv::aruco::detectMarkers` around line 535) and stamps white squares over their corners using `stamp_aruco_corners` so they become strong ORB features. In localization mode, when visual tracking is lost (`!ok`), if a known ArUco marker is visible in the scene, the system transitions from IDLE to MAPPING_MODE (`SLAM.DeactivateLocalizationMode()`). It simultaneously computes the exact camera world pose from ArUco PnP (`cv::aruco::estimatePoseSingleMarkers`). This was needed because standard visual relocalization can be slow or fail entirely; absolute physical markers give known global poses to instantly relocalize the robot without unpredictable feature matching.

### 2. External Pose Injection API
The `InjectPoseHint` call patches the ORB-SLAM3 tracker's `mLastFrame.pose` when the tracker is lost, overwriting it with a given SE3 hint. It simultaneously zeroes the tracker velocity and sets the tracking state to `Tracking::OK`. It enables tracking to instantly restart from an external absolute position (like one derived from ArUco) on the very next frame using `TrackWithMotionModel`, instead of trying to extrapolate from a stale velocity estimate or relying on standard visual relocalization. 
Exact function signature from the code:
`bool System::InjectPoseHint(const Sophus::SE3f& Tcw);`

### 3. Dual Operating Modes
- **`MODE_MAP` (Mapping mode):** Builds a fresh map dynamically from the camera feed. On exit, it saves `KeyFrameTrajectory.txt`, `MapPoints.txt`, `MarkerPositions.txt` (in SE3 format), and the full atlas (`room_map.osa`) to disk, and then automatically kicks off the `../map_builder.py` script. Selected via the command line flag `--map`.
- **`MODE_LOC` (Localization mode):** Loads a previously saved atlas (`.osa`) and engages `SLAM.ActivateLocalizationMode()`. It actively streams the real-time pose and gracefully recovers without modifying the persisted underlying map files on exit. Selected via `--loc <atlas_name>`.

### 4. ZMQ Pose Publisher
**Port:** 5557
**Message format:**
```cpp
    ss << "{\"seq\":" << seq << ",\"ts\":" << ts
       << ",\"ok\":" << (ok ? "true" : "false");
    if (ok) {
        Eigen::Vector3f t = cam_position(Tcw);
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

### 5. MarkerPositions.txt Output
It collects full SE3 observations per marker during mapping scenarios. Position values are simply averaged, and the marker rotations utilize a naive quaternion mean for combining rotation observations.
Format of each line:
`# marker_id tx ty tz qw qx qy qz num_observations`

## Bug Fixes

### 1. Atlas Serialization Crash

**What broke:** Calling `SLAM.Shutdown()` with `System.SaveAtlasToFile` set caused a segfault inside `Map::PreSave`, crashing the process before the atlas was written.

**Root cause:** `Map::PreSave` iterated directly over the live `mspMapPoints` set while calling `pMPi->EraseObservation(pKF)` inside the same loop; `EraseObservation` can call `SetBadFlag → EraseMapPoint`, which erases elements from `mspMapPoints` mid-iteration, invalidating the iterators.

**Fix** (`Map.cc` — `PreSave`, applied twice, once for the obs-cleanup pass and once for the backup pass):
```diff
- for(MapPoint* pMPi : mspMapPoints) {
+ std::vector<MapPoint*> vpMapPointsSnap(mspMapPoints.begin(), mspMapPoints.end());
+ for(MapPoint* pMPi : vpMapPointsSnap) {
```
The same snapshot pattern was applied to `mspKeyFrames` in the KeyFrame backup loop and to `mspMapPoints` in `PostLoad`.

**Why it mattered:** Without this fix, every `--map` session ended in a crash and produced a corrupt or missing `.osa` file, making atlas persistence and cross-session localization impossible.

### 2. Cross-Map Relocalization

**What broke:** After loading a saved atlas and switching to localization mode, `DetectRelocalizationCandidates` returned zero candidates even when BoW words were shared, so the tracker could never relocalize from the loaded map.

**Root cause:** Upstream `DetectRelocalizationCandidates` in `KeyFrameDatabase.cc` filtered relocalization candidates to only those whose `GetMap()` matched the `pMap` argument (the current active map). After `LoadAtlas`, the stored keyframes belong to a *different* (non-active) map object, so they were silently discarded even though their BoW descriptors matched the current frame perfectly.

**Fix** (`KeyFrameDatabase.cc` — `DetectRelocalizationCandidates`, final scoring loop):
```diff
- if (pKFi->GetMap() != pMap)
-     continue;
+ if (false) // disabled: map filter breaks relocalization after atlas load
+     continue;
```

**Why it mattered:** This was a silent failure — the inverted-file lookup succeeded and found word matches, but all candidates were thrown away before scoring, leaving the tracker perpetually lost; disabling the guard lets loaded-map keyframes participate in relocalization.

### 3. Shutdown Race Condition

**What broke:** `SLAM.Shutdown()` called `SaveAtlas` while `LocalMapping` and `LoopClosing` threads were still running, causing data races on shared map structures (keyframes, map points) and producing either a deadlock or a corrupted atlas file.

**Root cause:** Upstream `Shutdown` requested thread finish but did not wait for both threads to actually terminate before proceeding to serialization; the save could race against an in-progress bundle adjustment or loop-closing optimization still mutating the map.

**Fix** (`System.cc` — `Shutdown`):
```diff
+ while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()) {
+     usleep(5000);
+ }
  if (!mStrSaveAtlasToFile.empty()) {
      SaveAtlas(FileType::BINARY_FILE);
  }
```
A secondary guard stops any lingering GBA thread with `StopGBA()` and polls `isRunningGBA()` before proceeding to `SaveAtlas`.

**Why it mattered:** The hang or corrupt write on Ctrl+C made every mapping session unreliable; after this fix the atlas saves cleanly and the subsequent localization session loads a consistent map.

## Running

### Mapping mode
```bash
./slam_reader --map
```

### Localization mode
```bash
./slam_reader --loc <atlas_name>
```

## ZMQ Output
**Port:** 5557
**Message format:** JSON payload mapping to `{"seq":<seq>,"ts":<ts>,"ok":<true/false>,"x":<x>,"y":<y>,"z":<z>,"R":[<r00,r01,...>]}` (or `null` fields if `ok` is false).
**Frequency:** Published once per every unique frame obtained from the shared memory interface limit (`seq != last_seq`), scaling consistently to match the camera ingestion rate (typically ~30 fps).
