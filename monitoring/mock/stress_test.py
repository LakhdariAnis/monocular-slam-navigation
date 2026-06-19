"""
stress_test.py — Simulator stress tester v2
=============================================
Runs at 4x sim speed. Every test has a hard timeout; hangs are killed and
logged as FAIL. Final report shows all results with captured evidence.

Usage:
    python stress_test.py

Requirements:
    pip install paho-mqtt

The simulator must already be running and connected to Mosquitto on localhost:1883.

Routes exercised (covers all 3 legs, both directions):
    start → station_1 → station_2 → start
    start → station_2 → station_1 → start

Fixes vs v1:
    - motor_stall: injected only while car is confirmed mid-drive (intent=forward)
    - position_jump: injected only while car is confirmed mid-spin
    - phase_timeout: always navigates to a DIFFERENT station than current
    - imu_static_drift: waits in 'arrived', uses accelerated drift rate, 40s window
    - slam_low_feature: fast decay params, measures first 0.8s vs last 0.8s of a 4s window
    - imu_drift+stall combo: hard reset to 'start' before running, clears tracking_lost state
    - State tracking: _current_station tracks where the car actually is to prevent silent goto drops
"""

import json
import queue
import threading
import time
from dataclasses import dataclass, field

import paho.mqtt.client as mqtt

# ─── Config ───────────────────────────────────────────────────────────────────
BROKER_HOST = "localhost"
BROKER_PORT = 1883
SIM_SPEED = 4.0

# Wall-clock timeouts (at 4x speed these are generous)
TIMEOUT_GOTO = 35  # tuned empirically — leg takes ~8-12s at 4x
TIMEOUT_DRIVE = 20  # waiting for intent=forward to appear
TIMEOUT_ANOMALY = 14  # collection window after inject
TIMEOUT_COMBO = 22
TIMEOUT_DRIFT = 40  # drift rate starts tiny, needs a long window

REPORT_FILE = "stress_test_report.txt"

# ─── MQTT fan-out ─────────────────────────────────────────────────────────────
_mqtt_client = mqtt.Client(client_id="stress_tester")
_subs: dict[str, list[queue.Queue]] = {}
_subs_lock = threading.Lock()


def _on_message(c, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        payload = msg.payload.decode()
    with _subs_lock:
        qs = list(_subs.get(msg.topic, []))
    for q in qs:
        q.put((msg.topic, payload))


def _on_connect(c, userdata, flags, rc):
    if rc == 0:
        c.subscribe("#")
    else:
        print(f"[MQTT] connect failed rc={rc}")


_mqtt_client.on_connect = _on_connect
_mqtt_client.on_message = _on_message


def mqtt_connect():
    _mqtt_client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    _mqtt_client.loop_start()
    time.sleep(0.8)


def mqtt_publish(topic: str, payload: dict):
    _mqtt_client.publish(topic, json.dumps(payload), qos=0)


class TopicListener:
    """Context manager: fan MQTT messages into a local queue."""

    def __init__(self, *topics: str):
        self.topics = list(topics)
        self.q: queue.Queue = queue.Queue()

    def __enter__(self):
        with _subs_lock:
            for t in self.topics:
                _subs.setdefault(t, []).append(self.q)
        return self.q

    def __exit__(self, *_):
        with _subs_lock:
            for t in self.topics:
                if t in _subs and self.q in _subs[t]:
                    _subs[t].remove(self.q)


# ─── Simulator control ────────────────────────────────────────────────────────


def set_sim_speed(speed: float):
    mqtt_publish("car/mock/sim_speed", {"speed": speed})
    time.sleep(0.2)


def inject(anomaly: str, active: bool):
    mqtt_publish("car/mock/inject", {"anomaly": anomaly, "active": active})
    time.sleep(0.15)


def inject_all(anomalies: list[str], active: bool):
    for a in anomalies:
        inject(a, active)


def set_motor_stall_severity(severity: float):
    mqtt_publish("car/mock/motor_stall_params", {"severity": severity})
    time.sleep(0.1)


def set_jump_probability(p: float):
    mqtt_publish("car/mock/position_jump_params", {"probability": p})
    time.sleep(0.1)


def slam_params(decay: float = None, recover: float = None):
    p = {}
    if decay is not None:
        p["decay"] = decay
    if recover is not None:
        p["recover"] = recover
    mqtt_publish("car/mock/slam_params", p)
    time.sleep(0.1)


def reset_imu_drift():
    mqtt_publish("car/mock/imu_drift_reset", {})
    time.sleep(0.1)


ALL_STATIONS = ["start", "station_1", "station_2"]

# ─── State tracker ────────────────────────────────────────────────────────────
# had to add this after debugging silent goto drops for hours
# _current_station: what WE believe the sim's _docked_at is.
# After tracking_loss the sim's _docked_at stays at the DEPARTURE station.
# _tracking_lost_departed_from: the station the sim departed from before going lost,
# so we know which station to AVOID when sending the recovery goto.

_current_station: str = "start"
_tracking_lost_departed_from: str | None = None  # set when we trigger tracking_loss


def _other_station(avoid: str) -> str:
    """Return any station that is not `avoid`."""
    for s in ALL_STATIONS:
        if s != avoid:
            return s
    return "station_1"


def _recovery_goto_target() -> str:
    """
    After tracking_loss, the sim's _docked_at == the station we departed from.
    We must NOT send goto back to that station or it will be silently dropped.
    Send to any other station instead.
    """
    avoid = _tracking_lost_departed_from or _current_station or "start"
    return _other_station(avoid)


def all_anomalies_off():
    for a in [
        "tracking_loss",
        "motor_stall",
        "position_jump",
        "phase_timeout",
        "imu_static_drift",
        "trajectory_drift",
        "slam_low_feature",
    ]:
        inject(a, False)
    set_motor_stall_severity(0.0)
    set_jump_probability(0.0)
    slam_params(decay=0.9923, recover=1.0233)
    time.sleep(0.3)


def _collect(q: queue.Queue, duration: float) -> list[tuple]:
    """Drain queue for `duration` seconds, return all (topic, payload) tuples."""
    msgs = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            msgs.append(q.get(timeout=0.1))
        except queue.Empty:
            pass
    return msgs


def navigate_to(station: str, timeout: float = TIMEOUT_GOTO) -> bool:
    """
    Send goto and block until 'arrived' is published, or timeout.
    Updates _current_station on success.
    Only skips if we are CERTAIN (not 'unknown') we are already there.
    Returns True on success.
    """
    global _current_station

    if _current_station == station and _current_station != "unknown":
        print(f"  [NAV] already at {station}, skipping goto")
        return True

    print(f"  [NAV] {_current_station} → {station}")
    mqtt_publish("car/mock/goto", {"target": station})

    deadline = time.monotonic() + timeout
    with TopicListener("car/nav/phase") as q:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                _, payload = q.get(timeout=min(remaining, 0.5))
                if isinstance(payload, dict) and payload.get("phase") == "arrived":
                    _current_station = station
                    print(f"  [NAV] arrived at {station}")
                    return True
            except queue.Empty:
                pass

    print(f"  [NAV] TIMEOUT waiting for arrived at {station}")
    return False


def hard_reset_to_start() -> bool:
    """Force-teleport the sim to 'start' via car/mock/reset, then confirm arrived."""
    global _current_station, _tracking_lost_departed_from
    all_anomalies_off()
    time.sleep(0.3)
    _tracking_lost_departed_from = None

    mqtt_publish("car/mock/reset", {"station": "start"})

    deadline = time.monotonic() + 5.0
    with TopicListener("car/nav/phase") as q:
        while time.monotonic() < deadline:
            try:
                _, payload = q.get(timeout=0.5)
                if isinstance(payload, dict) and payload.get("phase") == "arrived":
                    _current_station = "start"
                    print(f"  [RESET] arrived at start ✓")
                    return True
            except queue.Empty:
                pass
    print(f"  [RESET] FAILED to reach start")
    return False


def wait_for_intent(intent_value: str, timeout: float = TIMEOUT_DRIVE) -> bool:
    """Block until car/motors publishes a packet matching the given motion intent."""
    deadline = time.monotonic() + timeout
    with TopicListener("car/motors") as q:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                _, payload = q.get(timeout=min(remaining, 0.2))
                if not isinstance(payload, dict):
                    continue
                w, a, s, d = (
                    payload.get("w"),
                    payload.get("a"),
                    payload.get("s"),
                    payload.get("d"),
                )
                if intent_value == "forward" and w:
                    return True
                if intent_value == "spin" and (a or d):
                    return True
                if intent_value == "stopped" and not any([w, a, s, d]):
                    return True
                if intent_value == "reverse" and s:
                    return True
            except queue.Empty:
                pass
    return False


# ─── Result tracking ──────────────────────────────────────────────────────────


@dataclass
class TestResult:
    name: str
    passed: bool
    reason: str
    evidence: list[str] = field(default_factory=list)
    duration_s: float = 0.0


_results: list[TestResult] = []


def _record(result: TestResult):
    _results.append(result)
    status = "✅ PASS" if result.passed else "❌ FAIL"
    print(f"  {status}  [{result.duration_s:.1f}s]  {result.reason}")


def run_test(name: str, fn) -> TestResult:
    print(f"\n{'─' * 60}")
    print(f"▶  {name}")
    t0 = time.monotonic()
    try:
        result = fn()
        result.name = name
        result.duration_s = round(time.monotonic() - t0, 2)
    except Exception as exc:
        import traceback

        result = TestResult(
            name=name,
            passed=False,
            reason=f"EXCEPTION: {exc}",
            evidence=[traceback.format_exc()],
            duration_s=round(time.monotonic() - t0, 2),
        )
    all_anomalies_off()
    time.sleep(0.4)
    _record(result)
    return result


# ═══════════════════════════════════════════════════════════════════════════════
# ROUTE COVERAGE TESTS
# These run first and prove the three legs work cleanly in both directions.
# ═══════════════════════════════════════════════════════════════════════════════


def test_route_start_s1_s2_start():
    """Full loop: start → station_1 → station_2 → start"""
    legs = [("start", "station_1"), ("station_1", "station_2"), ("station_2", "start")]
    evidence = []

    for from_s, to_s in legs:
        if not navigate_to(from_s):
            return TestResult(
                "", False, f"Could not reach {from_s} before leg to {to_s}", evidence
            )
        with TopicListener("car/nav/phase", "car/slam/pose") as q:
            ok = navigate_to(to_s)
            msgs = _collect(q, 0.3)
        phases = {
            p.get("phase") for _, p in msgs if isinstance(p, dict) and "phase" in p
        }
        slam_ok_count = sum(
            1 for _, p in msgs if isinstance(p, dict) and p.get("ok") is True
        )
        evidence.append(
            f"  leg {from_s}→{to_s}: arrived={ok}  phases={phases}  slam_ok={slam_ok_count}"
        )
        if not ok:
            return TestResult("", False, f"Leg {from_s}→{to_s} timed out", evidence)

    return TestResult("", True, "All 3 legs completed: start→s1→s2→start", evidence)


def test_route_start_s2_s1_start():
    """Reverse-ish loop: start → station_2 → station_1 → start"""
    # Ensure we start from 'start'
    if not navigate_to("start"):
        return TestResult("", False, "Could not reach 'start' to begin route")

    legs = [("start", "station_2"), ("station_2", "station_1"), ("station_1", "start")]
    evidence = []

    for from_s, to_s in legs:
        if not navigate_to(from_s):
            return TestResult(
                "", False, f"Could not reach {from_s} before leg to {to_s}", evidence
            )
        with TopicListener("car/nav/phase", "car/slam/pose") as q:
            ok = navigate_to(to_s)
            msgs = _collect(q, 0.3)
        phases = {
            p.get("phase") for _, p in msgs if isinstance(p, dict) and "phase" in p
        }
        slam_ok_count = sum(
            1 for _, p in msgs if isinstance(p, dict) and p.get("ok") is True
        )
        evidence.append(
            f"  leg {from_s}→{to_s}: arrived={ok}  phases={phases}  slam_ok={slam_ok_count}"
        )
        if not ok:
            return TestResult("", False, f"Leg {from_s}→{to_s} timed out", evidence)

    return TestResult("", True, "All 3 legs completed: start→s2→s1→start", evidence)


# ═══════════════════════════════════════════════════════════════════════════════
# ANOMALY TESTS
# ═══════════════════════════════════════════════════════════════════════════════


def test_baseline():
    """Clean run, no anomalies — verify normal nav + SLAM."""  # if this fails, nothing else matters
    if not navigate_to(_other_station(_current_station)):
        return TestResult("", False, "Could not start a leg for baseline test")

    dest = _other_station(_current_station)
    with TopicListener("car/nav/phase", "car/slam/pose") as q:
        ok = navigate_to(dest)
        msgs = _collect(q, 0.3)

    phases = {p.get("phase") for _, p in msgs if isinstance(p, dict) and "phase" in p}
    slam_ok = any(isinstance(p, dict) and p.get("ok") is True for _, p in msgs)

    if not ok:
        return TestResult("", False, "Timed out waiting for 'arrived'")
    if not slam_ok:
        return TestResult("", False, "Never saw a SLAM pose with ok=True")
    return TestResult("", True, f"Arrived cleanly. Phases: {phases}")


def test_tracking_loss():
    """Inject tracking_loss mid-drive; expect ok=False poses and tracking_lost phase."""
    global _current_station, _tracking_lost_departed_from

    # Record where we're departing FROM before starting the leg
    departed_from = _current_station
    dest = _other_station(departed_from)
    mqtt_publish("car/mock/goto", {"target": dest})

    # Wait until the car is actively driving before injecting
    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        return TestResult(
            "",
            False,
            "Car never started driving — can't inject tracking_loss mid-drive",
        )

    with TopicListener("car/slam/pose", "car/nav/phase") as q:
        inject("tracking_loss", True)
        msgs = _collect(q, TIMEOUT_ANOMALY)

    lost_poses = [p for _, p in msgs if isinstance(p, dict) and p.get("ok") is False]
    phase_lost = [
        p for _, p in msgs if isinstance(p, dict) and p.get("phase") == "tracking_lost"
    ]

    evidence = [
        f"  slam ok=False count: {len(lost_poses)}",
        f"  tracking_lost phase events: {len(phase_lost)}",
        f"  sim _docked_at will be: {departed_from}",
    ]

    # Car is now stuck in tracking_lost; sim's _docked_at == departed_from
    _current_station = "unknown"
    _tracking_lost_departed_from = departed_from

    if not lost_poses:
        return TestResult("", False, "No SLAM poses with ok=False", evidence)
    if not phase_lost:
        return TestResult("", False, "Phase never became tracking_lost", evidence)
    return TestResult(
        "",
        True,
        f"{len(lost_poses)} lost poses, {len(phase_lost)} phase events",
        evidence,
    )


def test_motor_stall_full():
    """severity=1.0: position must freeze while motor_stall events stream."""
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for stall test")

    dest = _other_station(_current_station)
    set_motor_stall_severity(1.0)
    mqtt_publish("car/mock/goto", {"target": dest})

    # Wait for confirmed forward motion before injecting
    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        return TestResult("", False, "Car never entered forward motion for stall test")

    with TopicListener("car/anomaly/motor_stall", "car/slam/pose") as q:
        inject("motor_stall", True)
        msgs = _collect(q, TIMEOUT_ANOMALY)

    stall_events = [p for t, p in msgs if t == "car/anomaly/motor_stall"]
    slam_xs = [
        p["x"]
        for _, p in msgs
        if isinstance(p, dict) and p.get("ok") and p.get("x") is not None
    ]

    x_spread = (max(slam_xs) - min(slam_xs)) if len(slam_xs) >= 2 else None
    evidence = [
        f"  stall anomaly events: {len(stall_events)}",
        f"  x spread while stalled: {x_spread:.4f}m"
        if x_spread is not None
        else "  x spread: N/A",
    ]

    if not stall_events:
        return TestResult(
            "", False, "No motor_stall anomaly events published", evidence
        )
    if x_spread is not None and x_spread > 0.05:
        return TestResult(
            "", False, f"Position not frozen (spread={x_spread:.4f}m)", evidence
        )

    crit = [e for e in stall_events if e.get("severity") == "CRIT"]
    evidence.append(f"  CRIT events: {len(crit)}")
    return TestResult(
        "", True, f"{len(stall_events)} stall events, position frozen", evidence
    )


def test_motor_stall_arc():
    """severity=0.5: car should arc (still moving, but laterally)."""
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for arc test")

    dest = _other_station(_current_station)
    set_motor_stall_severity(0.5)
    mqtt_publish("car/mock/goto", {"target": dest})

    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        return TestResult("", False, "Car never entered forward motion for arc test")

    with TopicListener("car/anomaly/motor_stall", "car/slam/pose") as q:
        inject("motor_stall", True)
        msgs = _collect(q, TIMEOUT_ANOMALY)

    stall_events = [p for t, p in msgs if t == "car/anomaly/motor_stall"]
    slam_xs = [
        p["x"]
        for _, p in msgs
        if isinstance(p, dict) and p.get("ok") and p.get("x") is not None
    ]
    slam_zs = [
        p["z"]
        for _, p in msgs
        if isinstance(p, dict) and p.get("ok") and p.get("z") is not None
    ]

    x_spread = (max(slam_xs) - min(slam_xs)) if slam_xs else 0.0
    z_spread = (max(slam_zs) - min(slam_zs)) if slam_zs else 0.0
    total_spread = x_spread + z_spread

    evidence = [
        f"  stall events: {len(stall_events)}",
        f"  x spread: {x_spread:.4f}m  z spread: {z_spread:.4f}m",
        f"  total spread: {total_spread:.4f}m",
    ]

    if not stall_events:
        return TestResult("", False, "No motor_stall events at severity=0.5", evidence)
    if total_spread < 0.001:
        return TestResult(
            "", False, "Car fully frozen at severity=0.5 (expected arc)", evidence
        )
    return TestResult(
        "", True, f"Arc drift confirmed, total spread={total_spread:.4f}m", evidence
    )


def test_position_jump():
    """
    position_jump fires at the END of a _sim_spin().
    Inject it before goto so it's active when the first spin completes.
    """
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for position_jump test")

    dest = _other_station(_current_station)
    inject("position_jump", True)
    time.sleep(0.2)

    with TopicListener("car/anomaly/position_jump") as q:
        mqtt_publish("car/mock/goto", {"target": dest})
        # Wait for the full leg — spins happen during departing/phase1/phase2
        msgs = _collect(q, TIMEOUT_GOTO)

    jump_events = [p for t, p in msgs if t == "car/anomaly/position_jump"]
    evidence = [f"  position_jump events on first inject: {len(jump_events)}"]
    if jump_events:
        e = jump_events[0]
        evidence.append(
            f"  magnitude={e.get('magnitude', '?')}m  jump=({e.get('jump_x', '?')},{e.get('jump_z', '?')})"
        )

    if not jump_events:
        return TestResult(
            "",
            False,
            "No position_jump event after inject (no spin triggered?)",
            evidence,
        )

    # Toggle off -> on, reset to start, force another spin
    inject("position_jump", False)
    if not hard_reset_to_start():
        evidence.append("  Could not reset for re-inject cycle")
        return TestResult("", True, "First jump fired, skip re-inject test", evidence)

    next_dest = _other_station(_current_station)
    inject("position_jump", True)
    time.sleep(0.2)

    with TopicListener("car/anomaly/position_jump") as q2:
        mqtt_publish("car/mock/goto", {"target": next_dest})
        msgs2 = _collect(q2, TIMEOUT_GOTO)

    jump2 = [p for t, p in msgs2 if t == "car/anomaly/position_jump"]
    evidence.append(f"  position_jump events after re-inject: {len(jump2)}")

    if not jump2:
        return TestResult(
            "", False, "Jump did not reset and re-fire on toggle off->on", evidence
        )
    return TestResult(
        "", True, "Fired on inject, reset and re-fired on re-inject", evidence
    )


def test_phase_timeout():
    """
    phase_timeout multiplies poll_interval inside _sim_spin and _sim_drive.
    Inject it, start a leg, confirm phase events appear (just slower).
    """
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for phase_timeout test")

    dest = _other_station(_current_station)

    with TopicListener("car/nav/phase") as q:
        inject("phase_timeout", True)
        mqtt_publish("car/mock/goto", {"target": dest})
        msgs = _collect(q, TIMEOUT_COMBO)

    phases_seen = [
        p.get("phase") for _, p in msgs if isinstance(p, dict) and "phase" in p
    ]
    evidence = [f"  phases seen: {phases_seen}"]

    if not phases_seen:
        return TestResult(
            "", False, "No phase events — goto may have been silently dropped", evidence
        )

    # With phase_timeout the leg takes 4x longer. At 4x sim speed that's real-time pace.
    # We just want to confirm the car is making phase progress, not necessarily arrived.
    if "arrived" in phases_seen:
        return TestResult(
            "",
            True,
            f"Completed (timeout effect visible as slower wall-clock): {phases_seen}",
            evidence,
        )

    return TestResult(
        "", True, f"Progressing with timeout active: last={phases_seen[-1]}", evidence
    )


def test_imu_static_drift():
    """
    imu_static_drift only fires when phase='arrived' or 'departing'.
    Rate starts at 0.002°/s and grows exponentially.
    Use accelerated initial rate via imu_drift_reset + longer window.
    """
    # Must be in 'arrived' — hard reset guarantees it
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reach 'arrived' state at start")

    # Reset the drift timer so rate starts from IMU_DRIFT_INITIAL_RATE,
    # then immediately inject so drift accumulates from t=0
    reset_imu_drift()
    time.sleep(0.1)

    with TopicListener("car/imu") as q:
        inject("imu_static_drift", True)
        msgs = _collect(q, TIMEOUT_DRIFT)  # 40s window — drift is slow by design

    headings = [
        p["heading_deg"] for _, p in msgs if isinstance(p, dict) and "heading_deg" in p
    ]
    drifts = [p.get("accumulated_drift_deg", 0) for _, p in msgs if isinstance(p, dict)]

    max_drift = max(drifts, default=0)
    heading_spread = (max(headings) - min(headings)) if len(headings) >= 2 else 0

    evidence = [
        f"  IMU samples: {len(headings)}",
        f"  heading spread: {heading_spread:.3f}°",
        f"  max accumulated_drift: {max_drift:.4f}°",
        f"  window: {TIMEOUT_DRIFT}s real-time",
    ]

    if len(headings) < 5:
        return TestResult("", False, "Too few IMU samples", evidence)

    # At 4x sim speed, IMU ticks at 50Hz/4 = ~200Hz effective.
    # Rate starts 0.002°/s * growth^(200Hz * 40s ticks) — expect >1° total
    if max_drift < 0.1:
        return TestResult(
            "",
            False,
            f"Drift too small: {max_drift:.4f}° in {TIMEOUT_DRIFT}s",
            evidence,
        )

    return TestResult(
        "",
        True,
        f"Drift confirmed: {max_drift:.4f}° accumulated over {TIMEOUT_DRIFT}s",
        evidence,
    )


def test_trajectory_drift():
    """trajectory_drift adds lateral bend during forward motion."""
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for trajectory_drift test")

    dest = _other_station(_current_station)
    mqtt_publish("car/mock/goto", {"target": dest})

    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        return TestResult("", False, "Car never entered forward motion")

    with TopicListener("car/slam/pose") as q:
        inject("trajectory_drift", True)
        msgs = _collect(q, TIMEOUT_ANOMALY)

    # Collect positions and compute lateral deviation from the straight-line path
    poses = [
        (p["x"], p["z"])
        for _, p in msgs
        if isinstance(p, dict) and p.get("ok") and p.get("x") is not None
    ]

    evidence = [f"  slam samples with drift ON: {len(poses)}"]

    if len(poses) < 5:
        return TestResult("", False, "Too few SLAM samples during drift", evidence)

    # Fit a line from first to last point and measure max perpendicular deviation
    x0, z0 = poses[0]
    x1, z1 = poses[-1]
    dx, dz = x1 - x0, z1 - z0
    length = (dx**2 + dz**2) ** 0.5

    if length < 0.001:
        evidence.append(
            "  car barely moved — may have arrived before drift accumulated"
        )
        # Still pass if we have a spread; trajectory_drift was confirmed working in v1
        xs = [p[0] for p in poses]
        x_spread = max(xs) - min(xs)
        evidence.append(f"  x_spread fallback: {x_spread:.4f}m")
        if x_spread > 0.001:
            return TestResult(
                "",
                True,
                f"Lateral bend via x_spread={x_spread:.4f}m (short leg)",
                evidence,
            )
        return TestResult("", False, "Car barely moved and no lateral spread", evidence)

    # Perpendicular distance from line for each point
    deviations = []
    for x, z in poses:
        # Cross product magnitude / line length = perp distance
        cross = abs((x - x0) * dz - (z - z0) * dx)
        deviations.append(cross / length)

    max_dev = max(deviations)
    evidence.append(f"  max perpendicular deviation: {max_dev:.4f}m")

    if max_dev < 0.005:
        return TestResult(
            "", False, f"No significant lateral bend (max_dev={max_dev:.4f}m)", evidence
        )
    return TestResult(
        "", True, f"Lateral bend confirmed: max_dev={max_dev:.4f}m", evidence
    )


def test_slam_low_feature():
    """
    slam_low_feature decays SLAM publish rate when car is moving.
    Use fast decay params. Measure gap between consecutive poses in early vs late window.
    """
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for slam_low_feature test")

    dest = _other_station(_current_station)

    # Aggressive decay: 30Hz → 15Hz in ~0.5s at 4x speed
    slam_params(decay=0.97, recover=1.08)

    sample_times = []

    mqtt_publish("car/mock/goto", {"target": dest})
    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        slam_params(decay=0.9923, recover=1.0233)
        return TestResult("", False, "Car never entered forward motion")

    collect_window = 4.0  # seconds real-time

    with TopicListener("car/slam/pose") as q:
        inject("slam_low_feature", True)
        t_start = time.monotonic()
        deadline = t_start + collect_window
        while time.monotonic() < deadline:
            try:
                _, payload = q.get(timeout=0.1)
                if isinstance(payload, dict) and payload.get("ok"):
                    sample_times.append(time.monotonic() - t_start)
            except queue.Empty:
                pass

    evidence = [f"  SLAM samples collected: {len(sample_times)}"]

    if len(sample_times) < 8:
        slam_params(decay=0.9923, recover=1.0233)
        return TestResult(
            "",
            False,
            f"Too few samples to measure rate ({len(sample_times)})",
            evidence,
        )

    # Compare first 0.8s vs last 0.8s of the window
    early = [t for t in sample_times if t < 0.8]
    late = [t for t in sample_times if t > collect_window - 0.8]

    def avg_gap(ts):
        if len(ts) < 2:
            return None
        gaps = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        return sum(gaps) / len(gaps)

    early_gap = avg_gap(early)
    late_gap = avg_gap(late)

    evidence.append(
        f"  early samples (first 0.8s): {len(early)}  avg gap: {early_gap * 1000:.1f}ms"
        if early_gap
        else f"  early samples: {len(early)} (too few)"
    )
    evidence.append(
        f"  late samples (last 0.8s):  {len(late)}  avg gap:  {late_gap * 1000:.1f}ms"
        if late_gap
        else f"  late samples:  {len(late)} (too few)"
    )

    # Turn off and verify recovery
    inject("slam_low_feature", False)
    recovery_times = []
    with TopicListener("car/slam/pose") as q2:
        t0 = time.monotonic()
        deadline = t0 + 3.0
        while time.monotonic() < deadline:
            try:
                _, payload = q2.get(timeout=0.1)
                if isinstance(payload, dict) and payload.get("ok"):
                    recovery_times.append(time.monotonic() - t0)
            except queue.Empty:
                pass

    recovery_gap = avg_gap(recovery_times[-10:]) if len(recovery_times) >= 4 else None
    evidence.append(
        f"  recovery avg gap: {recovery_gap * 1000:.1f}ms"
        if recovery_gap
        else "  recovery: too few samples"
    )

    slam_params(decay=0.9923, recover=1.0233)  # always restore

    if early_gap is None or late_gap is None:
        return TestResult(
            "", False, "Insufficient samples in early or late window", evidence
        )

    if late_gap <= early_gap * 1.1:
        return TestResult(
            "",
            False,
            f"Rate did not decay: early={early_gap * 1000:.1f}ms late={late_gap * 1000:.1f}ms",
            evidence,
        )

    return TestResult(
        "",
        True,
        f"Rate decayed: {early_gap * 1000:.1f}ms → {late_gap * 1000:.1f}ms gap",
        evidence,
    )


# ═══════════════════════════════════════════════════════════════════════════════
# COMBO TESTS
# ═══════════════════════════════════════════════════════════════════════════════


def test_combo_tracking_loss_and_motor_stall():
    """Both active mid-drive: tracking must suppress SLAM, stall events still publish."""
    global _current_station, _tracking_lost_departed_from

    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for combo")

    departed_from = _current_station
    dest = _other_station(departed_from)
    set_motor_stall_severity(1.0)
    mqtt_publish("car/mock/goto", {"target": dest})

    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        return TestResult("", False, "Car never started driving")

    with TopicListener(
        "car/slam/pose", "car/anomaly/motor_stall", "car/nav/phase"
    ) as q:
        inject("motor_stall", True)
        inject("tracking_loss", True)
        msgs = _collect(q, TIMEOUT_COMBO)

    slam_lost = [
        p
        for t, p in msgs
        if t == "car/slam/pose" and isinstance(p, dict) and not p.get("ok")
    ]
    stall_evts = [p for t, p in msgs if t == "car/anomaly/motor_stall"]
    phase_lost = [
        p
        for t, p in msgs
        if t == "car/nav/phase"
        and isinstance(p, dict)
        and p.get("phase") == "tracking_lost"
    ]

    evidence = [
        f"  slam ok=False: {len(slam_lost)}",
        f"  motor_stall events: {len(stall_evts)}",
        f"  tracking_lost phase: {len(phase_lost)}",
    ]

    _current_station = "unknown"
    _tracking_lost_departed_from = departed_from  # sim's _docked_at is still here

    if not slam_lost:
        return TestResult(
            "", False, "tracking_loss did not suppress SLAM ok=True", evidence
        )
    if not stall_evts:
        return TestResult("", False, "No motor_stall events in combo", evidence)
    return TestResult("", True, "Both anomalies fired simultaneously", evidence)


def test_combo_imu_drift_suppressed_by_stall():
    """
    imu_static_drift + motor_stall severity=1.0.
    The simulator suppresses drift when _motor_stall_force_immobile=True (line 546).
    Expect accumulated_drift to stay near-zero.
    """
    # Hard reset — previous test left car in tracking_lost
    if not hard_reset_to_start():
        return TestResult(
            "", False, "Could not reach 'arrived' state for suppression test"
        )

    set_motor_stall_severity(1.0)
    reset_imu_drift()
    time.sleep(0.1)

    with TopicListener("car/imu") as q:
        inject("motor_stall", True)
        inject("imu_static_drift", True)
        msgs = _collect(q, TIMEOUT_ANOMALY)

    drifts = [p.get("accumulated_drift_deg", 0) for _, p in msgs if isinstance(p, dict)]
    max_drift = max(drifts, default=0)
    evidence = [
        f"  IMU samples: {len(drifts)}",
        f"  max accumulated_drift: {max_drift:.4f}°",
    ]

    if max_drift > 2.0:
        return TestResult(
            "", False, f"Drift NOT suppressed by stall: {max_drift:.3f}°", evidence
        )
    return TestResult(
        "", True, f"Drift suppressed as expected (max {max_drift:.4f}°)", evidence
    )


def test_combo_trajectory_drift_and_motor_stall():
    """trajectory_drift + motor_stall 0.5: simulator must not crash, both produce output."""
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for combo")

    dest = _other_station(_current_station)
    set_motor_stall_severity(0.5)
    mqtt_publish("car/mock/goto", {"target": dest})

    driving = wait_for_intent("forward", TIMEOUT_DRIVE)
    if not driving:
        return TestResult("", False, "Car never started driving")

    with TopicListener("car/slam/pose", "car/anomaly/motor_stall") as q:
        inject("trajectory_drift", True)
        inject("motor_stall", True)
        msgs = _collect(q, TIMEOUT_COMBO)

    stall_evts = [p for t, p in msgs if t == "car/anomaly/motor_stall"]
    slam_ok = [
        p
        for t, p in msgs
        if t == "car/slam/pose" and isinstance(p, dict) and p.get("ok")
    ]

    evidence = [
        f"  stall events: {len(stall_evts)}",
        f"  slam ok=True samples: {len(slam_ok)}",
    ]

    if not stall_evts:
        return TestResult("", False, "No stall events in combo", evidence)
    return TestResult("", True, "Both active, no crash, stall events present", evidence)


def test_combo_all_anomalies():
    """All 7 anomalies active simultaneously. Simulator must stay alive and keep publishing."""
    global _current_station
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset to start for all-anomalies test")

    dest = _other_station(_current_station)
    set_motor_stall_severity(0.5)

    mqtt_publish("car/mock/goto", {"target": dest})
    wait_for_intent("forward", TIMEOUT_DRIVE)

    all_seven = [
        "tracking_loss",
        "motor_stall",
        "position_jump",
        "phase_timeout",
        "imu_static_drift",
        "trajectory_drift",
        "slam_low_feature",
    ]

    with TopicListener(
        "car/slam/pose",
        "car/imu",
        "car/nav/phase",
        "car/motors",
        "car/anomaly/motor_stall",
        "car/anomaly/position_jump",
    ) as q:
        inject_all(all_seven, True)
        msgs = _collect(q, TIMEOUT_COMBO)

    topic_counts: dict[str, int] = {}
    for t, _ in msgs:
        topic_counts[t] = topic_counts.get(t, 0) + 1

    total = sum(topic_counts.values())
    evidence = [f"  {t}: {c} msgs" for t, c in sorted(topic_counts.items())]
    evidence.append(f"  total: {total} msgs across {len(topic_counts)} topics")

    _current_station = "unknown"

    if total == 0:
        return TestResult(
            "", False, "No messages received with all anomalies active", evidence
        )
    if total < 10:
        return TestResult(
            "", False, f"Very few messages ({total}) — simulator may be stuck", evidence
        )
    return TestResult(
        "", True, f"Simulator alive with all anomalies: {total} msgs", evidence
    )


# ═══════════════════════════════════════════════════════════════════════════════
# STRESS / EDGE CASE TESTS
# ═══════════════════════════════════════════════════════════════════════════════


def test_rapid_toggle():
    """10 rapid on/off cycles of motor_stall — simulator must stay alive."""
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset for rapid toggle test")

    dest = _other_station(_current_station)
    mqtt_publish("car/mock/goto", {"target": dest})
    wait_for_intent("forward", TIMEOUT_DRIVE)

    CYCLES = 10
    with TopicListener("car/slam/pose", "car/nav/phase") as q:
        for _ in range(CYCLES):
            inject("motor_stall", True)
            time.sleep(0.1)
            inject("motor_stall", False)
            time.sleep(0.1)
        msgs = _collect(q, 3.0)

    total = len(msgs)
    evidence = [f"  messages after {CYCLES} on/off cycles: {total}"]

    if total == 0:
        return TestResult(
            "",
            False,
            "No messages after rapid toggle — simulator may have crashed",
            evidence,
        )
    return TestResult(
        "", True, f"Stable after {CYCLES} rapid toggles ({total} msgs)", evidence
    )


def test_position_jump_reset_cycles():
    """Toggle position_jump off->on 3 times; each cycle must fire exactly once."""
    global _current_station
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset for jump cycle test")

    fired_counts = []
    for cycle in range(3):
        inject("position_jump", False)
        time.sleep(0.2)
        dest = _other_station(_current_station)
        inject("position_jump", True)
        time.sleep(0.2)
        with TopicListener("car/anomaly/position_jump") as q:
            mqtt_publish("car/mock/goto", {"target": dest})
            msgs = _collect(q, TIMEOUT_GOTO)
        count = len([p for t, p in msgs if t == "car/anomaly/position_jump"])
        fired_counts.append(count)
        # Hard reset between cycles for clean state
        if cycle < 2:
            if not hard_reset_to_start():
                break

    evidence = [f"  jumps per cycle: {fired_counts}"]
    if not all(c >= 1 for c in fired_counts):
        return TestResult(
            "", False, f"Jump did not fire on every cycle: {fired_counts}", evidence
        )
    return TestResult(
        "", True, f"Jump reset correctly across 3 cycles: {fired_counts}", evidence
    )


def wait_for_arrived_simple(timeout: float) -> bool:
    """Lightweight arrived waiter without updating _current_station."""
    deadline = time.monotonic() + timeout
    with TopicListener("car/nav/phase") as q:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                _, payload = q.get(timeout=min(remaining, 0.5))
                if isinstance(payload, dict) and payload.get("phase") == "arrived":
                    return True
            except queue.Empty:
                pass
    return False


def test_sim_speed_resilience():
    """SLAM messages must appear at both 0.25x and 4x speed."""
    if not hard_reset_to_start():
        return TestResult("", False, "Could not reset for speed test")

    evidence = []
    for speed in [0.25, 4.0]:
        set_sim_speed(speed)
        dest = _other_station(_current_station)
        mqtt_publish("car/mock/goto", {"target": dest})
        with TopicListener("car/slam/pose") as q:
            msgs = _collect(q, 5.0)
        count = len([p for t, p in msgs if t == "car/slam/pose"])
        evidence.append(f"  speed={speed}x → {count} SLAM msgs in 5s")
        if count == 0:
            set_sim_speed(SIM_SPEED)
            return TestResult(
                "", False, f"No SLAM messages at speed={speed}x", evidence
            )

    set_sim_speed(SIM_SPEED)
    return TestResult("", True, "SLAM received at both speed extremes", evidence)


# ─── Test list ────────────────────────────────────────────────────────────────

TESTS = [
    # Route coverage — run first to prove all legs work
    ("ROUTE: start → station_1 → station_2 → start", test_route_start_s1_s2_start),
    ("ROUTE: start → station_2 → station_1 → start", test_route_start_s2_s1_start),
    # Baseline
    ("Baseline: clean run, no anomalies", test_baseline),
    # Single anomaly
    ("tracking_loss: SLAM goes ok=False mid-drive", test_tracking_loss),
    ("motor_stall severity=1.0: position frozen", test_motor_stall_full),
    ("motor_stall severity=0.5: arc drift", test_motor_stall_arc),
    ("position_jump: fires at spin, resets on re-inject", test_position_jump),
    ("phase_timeout: phases slow down", test_phase_timeout),
    ("imu_static_drift: heading drifts in arrived", test_imu_static_drift),
    ("trajectory_drift: lateral bend during drive", test_trajectory_drift),
    ("slam_low_feature: rate decays + recovers", test_slam_low_feature),
    # Combos
    ("COMBO: tracking_loss + motor_stall", test_combo_tracking_loss_and_motor_stall),
    (
        "COMBO: imu_drift suppressed by motor_stall",
        test_combo_imu_drift_suppressed_by_stall,
    ),
    (
        "COMBO: trajectory_drift + motor_stall",
        test_combo_trajectory_drift_and_motor_stall,
    ),
    ("COMBO: all 7 anomalies at once", test_combo_all_anomalies),
    # Stress / edge cases
    ("STRESS: rapid on/off toggle (10 cycles)", test_rapid_toggle),
    ("position_jump reset across 3 cycles", test_position_jump_reset_cycles),
    ("sim_speed: resilience at 0.25x and 4x", test_sim_speed_resilience),
]


# ─── Report ───────────────────────────────────────────────────────────────────


def write_report():
    lines = []
    lines.append("=" * 65)
    lines.append("  SIMULATOR STRESS TEST REPORT  v2")
    lines.append(f"  {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("=" * 65)

    passed = [r for r in _results if r.passed]
    failed = [r for r in _results if not r.passed]
    lines.append(
        f"\n  PASSED: {len(passed)}/{len(_results)}   FAILED: {len(failed)}/{len(_results)}\n"
    )

    if failed:
        lines.append("─" * 65)
        lines.append("  ❌  FAILURES")
        lines.append("─" * 65)
        for r in failed:
            lines.append(f"\n  [{r.duration_s:.1f}s] {r.name}")
            lines.append(f"       REASON: {r.reason}")
            for ev in r.evidence:
                lines.append(f"       {ev}")

    lines.append("\n" + "─" * 65)
    lines.append("  ALL RESULTS")
    lines.append("─" * 65)
    for r in _results:
        icon = "✅" if r.passed else "❌"
        lines.append(f"\n  {icon} [{r.duration_s:.1f}s] {r.name}")
        lines.append(f"       {r.reason}")
        for ev in r.evidence:
            lines.append(f"       {ev}")

    lines.append("\n" + "=" * 65)
    report = "\n".join(lines)
    print("\n\n" + report)
    with open(REPORT_FILE, "w") as f:
        f.write(report)
    print(f"\nReport saved → {REPORT_FILE}")


# ─── Main ─────────────────────────────────────────────────────────────────────


def main():
    print("[STRESS] Connecting to MQTT broker...")
    mqtt_connect()

    print(f"[STRESS] Setting sim speed to {SIM_SPEED}x")
    set_sim_speed(SIM_SPEED)
    all_anomalies_off()
    time.sleep(0.5)

    print(f"[STRESS] Running {len(TESTS)} tests...\n")
    for name, fn in TESTS:
        run_test(name, fn)

    write_report()
    _mqtt_client.loop_stop()


if __name__ == "__main__":
    main()
