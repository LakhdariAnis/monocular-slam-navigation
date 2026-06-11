# Autonomous Car: Spinning & SLAM Recovery Protocol

This document outlines the testing, findings, and final protocol for handling 90-degree rotations in the autonomous car's navigation framework. It is intended to be used as a reference to upgrade the main pathfinding scripts.

## 1. The Problem
When the car reaches navigation waypoints (like an elbow) and needs to rotate 90 degrees, several hardware and software limitations clash:
* **Camera Blur & SLAM Loss:** Spinning too fast (e.g., 60% motor power) physically blurs the camera feed, causing ORB-SLAM3 to drop frames, lose tracking, and eventually snap to incorrect locations in the map (often teleporting the car 0.5m+ away).
* **Motor Dead-zones:** Yellow TT DC motors physically stall or jitter at extremely low speeds. We cannot rely on ultra-slow turning to preserve SLAM frames without the car getting stuck.
* **Controller Oscillation:** Tests with complex speed-adaptive controllers (easing, pulsing, IMU feedback loops) caused jerky movements because the minimum torque necessary to move the car changes drastically depending on surface friction.

## 2. What We Tested & Learned
We ran comprehensive tests via the `spin_debug.py` script:
1. **Pulsing/Coasting:** Stop-and-go movements worked but induced heavy jitter.
2. **Dynamic Ease-In/Out (IMU Loop):** Adjusting speeds continuously using IMU `deg/s` rate caused heavy oscillations between stalling and spinning too quickly.
3. **Low Constant Speed (The Sweet Spot):** We found that **30% constant speed** provides just enough torque to spin cleanly without stalling, and is slow enough that ORB-SLAM3 minimizes lost frames. At 30%, SLAM still drifts slightly during the spin, but stays within an acceptable margin (< 0.10m).

## 3. The "Shoot & Recover" Protocol
To perform a robust 90° spin safely, navigation scripts (like `path_finding.py`) must decouple the rotation phase from SLAM entirely, treating SLAM as a system to "recover" after the spin finishes.

### Step 1: Record Pre-Spin Position
Before starting the motors, log the absolute last known valid SLAM coordinates `(x, z)`.

### Step 2: Spin Blindly using IMU
Send a constant **30% power** command and track the exact heading delta directly from the IMU stream. Ignore any SLAM updates temporarily.
* *Tolerance:* Stop motors when the IMU determines you have turned ~84° (leaving ~6° for mechanical coasting leeway).

### Step 3: The 3-Second SLAM Settle
Once the motors stop, keep the car completely stationary for up to **3.0 seconds**.
ORB-SLAM3 requires stationary frames to cleanly execute its Relocalization / ArUco recovery module and clear tracking errors. Wait for a *fresh* frame to come in.

### Step 4: Drift Sanity Check
Check the distance between your pre-spin recorded position vs the new, fresh post-spin position.
* **If `drift <= 0.15m`**: The spin was successful. SLAM is accurately locked.
* **If `drift > 0.15m`**: SLAM has relocalized to the totally wrong area in the map (origin snapping). **Reject the reading.**

### Step 5: Reverse Relocalization Fallback
If SLAM failed the sanity check or never reappeared:
1. Drive the car **in reverse for ~0.4s at 40% speed**.
2. Stop and wait 3.0 seconds again.
3. Validate drift again. 
*(Reasoning: Reversing shows the camera back towards the original wall geometry, providing a wider field of known features for SLAM to latch onto).*


## 4. Integration into Navigation (`path_finding.py`)

Here is how you can adapt this protocol into the main logic of `path_finding.py` once you are ready.

1. Ensure you pull the isolated `recover_slam()` and `wait_for_fresh_slam()` functions from the `spin_debug.py` prototype into the main `path_finding.py` file.
2. Structure the navigation command flow like this:

```python
# 1. Grab Pre-Spin Position
pre_spin_pos = slam.get()

# 2. Spin tracking IMU (Make sure _spin_delta uses 30 speed)
_spin_delta(degrees=90, turn_left=turn_left, imu=imu)
 
# 3. Recover SLAM
print("Waiting for SLAM recovery...")
spin_end_time = time.time()
recovered_pos = recover_slam(pre_spin_pos, spin_end_time)

if recovered_pos is not None:
    print(f"✓ Ready to continue. Current pos: {recovered_pos}")
    # 4. Continue to Standoff using the safe position
    _drive_to("standoff", standoff_x, standoff_z, ...)
else:
    print("FATAL: SLAM lost completely.")
    _stop()
```
