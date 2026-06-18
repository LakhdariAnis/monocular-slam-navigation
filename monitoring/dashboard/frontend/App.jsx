// ─────────────────────────────────────────────
    // IMPORTS from global UMD bundles
    // ─────────────────────────────────────────────
    const {
      useState, useEffect, useRef, useCallback,
    } = React;

    const {
      ResponsiveContainer, ScatterChart, Scatter, LineChart, Line,
      XAxis, YAxis, CartesianGrid, Tooltip, Cell,
    } = Recharts;

    // ─────────────────────────────────────────────
    // CONSTANTS
    // ─────────────────────────────────────────────
    const API_BASE = "http://localhost:8000";
    const WS_URL   = "ws://localhost:8000/ws";

    const MAX_PATH_POINTS = 500;
    const MAX_CHART_POINTS = 200;
    const POSITION_JUMP_MAGNITUDE = 2.0;

    const ANOMALY_TYPES = [
      { key: "tracking_loss",    label: "Tracking Loss",     color: "#ff6b6b" },
      { key: "motor_stall",      label: "Motor Stall",       color: "#7dd3fc" },
      { key: "position_jump",    label: "Position Jump",     color: "#7dd3fc" },
      { key: "imu_static_drift", label: "IMU Static Drift",  color: "#7dd3fc" },
      { key: "slam_low_feature", label: "Low Feature Room",  color: "#facc15" },
    ];

    const PHASE_COLORS = {
      departing: { bg: "rgba(136,180,204,0.2)", border: "rgba(136,180,204,0.3)", text: "#88b4cc" },
      phase1:    { bg: "rgba(125,211,252,0.2)", border: "rgba(125,211,252,0.3)", text: "#7dd3fc" },
      phase2:    { bg: "rgba(125,211,252,0.25)", border: "rgba(125,211,252,0.35)", text: "#7dd3fc" },
      phase3:    { bg: "rgba(200,160,240,0.2)", border: "rgba(200,160,240,0.3)", text: "#c8a0f0" },
      aligning:  { bg: "rgba(200,234,255,0.15)", border: "rgba(200,234,255,0.3)", text: "#c8eaff" },
      arrived:   { bg: "rgba(74,222,128,0.15)", border: "rgba(74,222,128,0.3)", text: "#4ade80" },
      init:      { bg: "rgba(26,36,56,0.6)", border: "rgba(42,58,72,0.5)", text: "#a0b4c4" },
    };

const STATIONS = {
  start:     { x: -0.1155, z: -0.2249, standoff: [-0.1155,  0.0551], orientation: "-Z Wall" },
  station_1: { x:  0.6900, z:  0.5521, standoff: [ 0.3900,  0.5521], orientation: "+X Wall" },
  station_2: { x: -0.0303, z:  1.5225, standoff: [-0.0303,  1.2225], orientation: "+Z Wall" },
};

const ROUTE = [
  ["start",     "station_1"],
  ["station_1", "station_2"],
  ["station_2", "start"],
];

function computeElbow(fromName, toName) {
  const src = STATIONS[fromName];
  const dst = STATIONS[toName];
  const [cx, cz] = fromName === "start" ? [src.x, src.z] : src.standoff;
  const [sx, sz] = dst.standoff;
  let ex, ez;
  if (dst.orientation === "+X Wall" || dst.orientation === "-X Wall") {
    ex = cx; ez = sz;
  } else {
    ex = sx; ez = cz;
  }
  return { elbow: [ex, ez], standoff: [sx, sz], target: [dst.x, dst.z] };
}

    // ─────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────
    function fmtTime(ts) {
      if (!ts) return "";
      const d = new Date(typeof ts === "number" ? ts * 1000 : ts);
      if (isNaN(d.getTime())) return "";
      return d.toLocaleTimeString("en-GB", { hour12: false });
    }

    function fmtDuration(seconds) {
      if (seconds < 60) return `${Math.round(seconds)}s`;
      const m = Math.floor(seconds / 60);
      const s = Math.round(seconds % 60);
      return `${m}m${s}s`;
    }

    // ─────────────────────────────────────────────
    // CUSTOM HOOKS
    // ─────────────────────────────────────────────

    function useWebSocket() {
      const [connected, setConnected] = useState(false);
      const [snapshot, setSnapshot]   = useState({});
      const wsRef  = useRef(null);
      const retry  = useRef(null);

      const connect = useCallback(() => {
        if (wsRef.current && wsRef.current.readyState <= 1) return;

        const ws = new WebSocket(WS_URL);
        wsRef.current = ws;

        ws.onopen = () => {
          setConnected(true);
          if (retry.current) { clearTimeout(retry.current); retry.current = null; }
        };

        ws.onmessage = (evt) => {
          try {
            const data = JSON.parse(evt.data);
            setSnapshot(data);
          } catch {}
        };

        ws.onclose = () => {
          setConnected(false);
          retry.current = setTimeout(connect, 2000);
        };

        ws.onerror = () => {
          ws.close();
        };
      }, []);

      useEffect(() => {
        connect();
        return () => {
          if (wsRef.current) wsRef.current.close();
          if (retry.current) clearTimeout(retry.current);
        };
      }, [connect]);

      return { connected, snapshot };
    }

    function useHistory(endpoint) {
      const [data, setData]   = useState([]);
      const loaded = useRef(false);

      useEffect(() => {
        if (loaded.current) return;
        loaded.current = true;
        fetch(`${API_BASE}${endpoint}`)
          .then(r => r.json())
          .then(rows => setData(Array.isArray(rows) ? rows : []))
          .catch(() => {});
      }, [endpoint]);

      return [data, setData];
    }

    // ─────────────────────────────────────────────
    // COMPONENTS
    // ─────────────────────────────────────────────

    // ── Status Badge ──
    function StatusBadge({ color, shadow, children }) {
      return (
        <div className="glass-panel rounded-full px-4 py-1.5 flex items-center gap-3">
          <div
            className="w-2 h-2 rounded-full"
            style={{ backgroundColor: color, boxShadow: `0 0 8px ${shadow || color}` }}
          />
          <span className="font-label text-xs text-on-surface tracking-wider">{children}</span>
        </div>
      );
    }

    // ── Status Bar ──
    function StatusBar({ slam, phase, motors, slamSeq, wsConnected }) {
      const slamOk = slam?.ok;
      let slamLabel, slamColor;
      if (slamOk === true)       { slamLabel = "SLAM TRACKING: OK";    slamColor = "#4ade80"; }
      else if (slamOk === false) { slamLabel = "SLAM TRACKING: LOST";  slamColor = "#ff6b6b"; }
      else                       { slamLabel = "SLAM TRACKING: RELOC"; slamColor = "#facc15"; }

      const phaseLabel = phase?.phase ? phase.phase.toUpperCase().replace(/_/g, " ") : "—";

      const totalPwm = motors?.total ?? 0;
      const innerPwm = motors?.inner ?? 0;
      const motorRunning = totalPwm > 0;
      const motorLabel = motorRunning
        ? `MOTOR: RUNNING (${totalPwm}/${innerPwm})`
        : "MOTOR: STOPPED";

      return (
        <header id="status-bar" className="flex flex-wrap gap-3 items-center">
          <StatusBadge color={slamColor} shadow={slamColor}>{slamLabel}</StatusBadge>
          <StatusBadge color="#7dd3fc" shadow="#7dd3fc">PHASE: {phaseLabel}</StatusBadge>
          <StatusBadge color={motorRunning ? "#88b4cc" : "#4a6070"} shadow={motorRunning ? "#88b4cc" : "#4a6070"}>
            {motorLabel}
          </StatusBadge>
          {slamSeq != null && (
            <div className="glass-panel rounded-full px-4 py-1.5 flex items-center gap-2">
              <span className="font-label text-xs text-on-surface-variant">SEQ</span>
              <span className="font-label text-xs text-secondary">{slamSeq}</span>
            </div>
          )}
          <div className="glass-panel rounded-full px-4 py-1.5 flex items-center gap-2 ml-auto">
            <span className="font-label text-xs text-on-surface-variant">WS</span>
            <div
              className={`w-2 h-2 rounded-full ${wsConnected ? "pulse-green" : ""}`}
              style={{
                backgroundColor: wsConnected ? "#22c55e" : "#ff6b6b",
                boxShadow: wsConnected ? "0 0 8px #22c55e" : "0 0 8px #ff6b6b",
              }}
            />
          </div>
        </header>
      );
    }

    // ── Position Path Chart ──
    function PositionPathChart({ historyData, liveSlam, slamRateStatus }) {
      const [points, setPoints] = useState([]);

      useEffect(() => {
        if (historyData.length > 0) {
          const pts = historyData
            .filter(r => r.x != null && r.z != null)
            .map(r => ({ x: r.x, z: r.z }));
          setPoints(pts.slice(-MAX_PATH_POINTS));
        }
      }, [historyData]);

      useEffect(() => {
        if (liveSlam && liveSlam.ok && liveSlam.x != null && liveSlam.z != null) {
          setPoints(prev => {
            const next = [...prev, { x: liveSlam.x, z: liveSlam.z }];
            return next.slice(-MAX_PATH_POINTS);
          });
        }
      }, [liveSlam]);

      const currentPos = points.length > 0 ? points[points.length - 1] : null;

      // Custom dot to show trail
      const renderDot = (props) => {
        const { cx, cy, index } = props;
        if (cx == null || cy == null) return null;
        const isLast = index === points.length - 1;
        const opacity = 0.3 + 0.7 * (index / Math.max(points.length - 1, 1));
        const degraded = slamRateStatus && slamRateStatus !== "ok";
        const normalColor = "#7dd3fc";
        const lastColor = "#c8eaff";
        const degradedColor = "#facc15";
        const dotColor = isLast
          ? (degraded ? degradedColor : lastColor)
          : (degraded ? degradedColor : normalColor);
        return (
          <circle
            key={index}
            cx={cx}
            cy={cy}
            r={isLast ? 5 : 1.5}
            fill={dotColor}
            fillOpacity={opacity}
            filter={isLast ? "url(#glowFilter)" : undefined}
          />
        );
      };

      return (
        <div id="position-path-chart" className="lg:col-span-8 glass-panel rounded-xl overflow-hidden flex flex-col relative min-h-[400px]">
          <div className="p-4 border-b border-outline-variant flex justify-between items-center z-10"
               style={{ background: "rgba(32, 44, 66, 0.3)" }}>
            <h3 className="font-headline text-lg text-primary glow-text-primary">Position Path (X vs Z)</h3>
            <span className="font-label text-xs text-on-surface-variant">10-minute history | Live</span>
          </div>

          <div className="absolute inset-0 top-[60px] chart-grid-bg opacity-80" style={{ background: "#0a0e1a" }} />

          <div className="flex-1 relative z-[1] min-h-[340px]">
            <ResponsiveContainer width="100%" height="100%">
              <ScatterChart margin={{ top: 20, right: 30, bottom: 20, left: 20 }}>
                <defs>
                  <filter id="glowFilter">
                    <feGaussianBlur stdDeviation="3" result="coloredBlur" />
                    <feMerge>
                      <feMergeNode in="coloredBlur" />
                      <feMergeNode in="SourceGraphic" />
                    </feMerge>
                  </filter>
                </defs>
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(125,211,252,0.06)" />
                <XAxis
                  type="number" dataKey="x" name="X"
                  stroke="rgba(125,211,252,0.2)"
                  tick={{ fill: "#a0b4c4", fontSize: 10 }}
                  label={{ value: "X", position: "insideBottomRight", fill: "#a0b4c4", fontSize: 11 }}
                />
                <YAxis
                  type="number" dataKey="z" name="Z"
                  stroke="rgba(125,211,252,0.2)"
                  tick={{ fill: "#a0b4c4", fontSize: 10 }}
                  label={{ value: "Z", position: "insideTopLeft", fill: "#a0b4c4", fontSize: 11 }}
                />
                <Tooltip
                  contentStyle={{
                    background: "rgba(15, 21, 36, 0.9)",
                    border: "1px solid rgba(125, 211, 252, 0.2)",
                    borderRadius: "8px",
                    color: "#e0e8f0",
                    fontSize: "12px",
                  }}
                  formatter={(val) => val?.toFixed(4)}
                />
                <Scatter
                  data={points}
                  line={{ stroke: (slamRateStatus && slamRateStatus !== "ok") ? "#facc15" : "#7dd3fc", strokeWidth: 2, filter: "drop-shadow(0 0 6px rgba(125,211,252,0.6))" }}
                  lineType="joint"
                  shape={renderDot}
                  isAnimationActive={false}
                />
              </ScatterChart>
            </ResponsiveContainer>
          </div>

          {currentPos && (
            <div className="absolute bottom-4 left-4 z-10 glass-panel-elevated p-3 rounded-lg flex flex-col gap-2">
              <div className="text-xs font-label text-on-surface-variant">CURRENT POS</div>
              <div className="font-display text-lg text-primary flex gap-4 glow-text-primary">
                <span>X: {currentPos.x?.toFixed(4)}</span>
                <span>Z: {currentPos.z?.toFixed(4)}</span>
              </div>
            </div>
          )}
        </div>
      );
    }

    // ── IMU Heading Chart ──
    function IMUHeadingChart({ historyData, liveImu }) {
      const [points, setPoints] = useState([]);

      useEffect(() => {
        if (historyData.length > 0) {
          const pts = historyData.map(r => ({
            time: fmtTime(r.ts),
            heading: r.heading_deg,
          }));
          setPoints(pts.slice(-MAX_CHART_POINTS));
        }
      }, [historyData]);

      useEffect(() => {
        if (liveImu && liveImu.heading_deg != null) {
          setPoints(prev => {
            const next = [...prev, {
              time: fmtTime(liveImu.ts),
              heading: liveImu.heading_deg,
            }];
            return next.slice(-MAX_CHART_POINTS);
          });
        }
      }, [liveImu]);

      const currentHeading = points.length > 0 ? points[points.length - 1].heading : null;

      return (
        <div id="imu-heading-chart" className="glass-panel p-4 rounded-xl flex-1 flex flex-col">
          <h3 className="font-label text-xs text-on-surface-variant mb-2 flex items-center gap-2">
            <span className="material-symbols-outlined text-sm">explore</span>
            IMU Heading
          </h3>
          <div className="flex-1 relative min-h-[100px]">
            {currentHeading != null && (
              <div className="absolute top-0 right-0 font-display text-xl text-secondary z-10">
                {Math.round(currentHeading)}°
              </div>
            )}
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={points} margin={{ top: 5, right: 5, bottom: 5, left: -20 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(125,211,252,0.06)" />
                <XAxis
                  dataKey="time" stroke="rgba(125,211,252,0.2)"
                  tick={{ fill: "#a0b4c4", fontSize: 9 }}
                  interval="preserveStartEnd"
                  minTickGap={50}
                />
                <YAxis
                  domain={[0, 360]}
                  stroke="rgba(125,211,252,0.2)"
                  tick={{ fill: "#a0b4c4", fontSize: 9 }}
                  ticks={[0, 90, 180, 270, 360]}
                />
                <Tooltip
                  contentStyle={{
                    background: "rgba(15, 21, 36, 0.9)",
                    border: "1px solid rgba(125, 211, 252, 0.2)",
                    borderRadius: "8px",
                    color: "#e0e8f0",
                    fontSize: "12px",
                  }}
                />
                <Line
                  type="monotone" dataKey="heading" stroke="#88b4cc" strokeWidth={2}
                  dot={false} isAnimationActive={false}
                  filter="drop-shadow(0 0 4px rgba(136,180,204,0.5))"
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>
      );
    }

    // ── Motor PWM Chart ──
    function MotorPWMChart({ historyData, liveMotors }) {
      const [points, setPoints] = useState([]);

      useEffect(() => {
        if (historyData.length > 0) {
          const pts = historyData.map(r => ({
            time: fmtTime(r.ts),
            total: r.total,
            inner: r.inner,
          }));
          setPoints(pts.slice(-MAX_CHART_POINTS));
        }
      }, [historyData]);

      useEffect(() => {
        if (liveMotors && liveMotors.total != null) {
          setPoints(prev => {
            const next = [...prev, {
              time: fmtTime(liveMotors.ts),
              total: liveMotors.total,
              inner: liveMotors.inner,
            }];
            return next.slice(-MAX_CHART_POINTS);
          });
        }
      }, [liveMotors]);

      const currentTotal = points.length > 0 ? points[points.length - 1].total : null;

      return (
        <div id="motor-pwm-chart" className="glass-panel p-4 rounded-xl flex-1 flex flex-col">
          <h3 className="font-label text-xs text-on-surface-variant mb-2 flex items-center gap-2">
            <span className="material-symbols-outlined text-sm">speed</span>
            Motor PWM (Total/Inner)
          </h3>
          <div className="flex-1 relative min-h-[100px]">
            {currentTotal != null && (
              <div className="absolute top-0 right-0 font-display text-xl text-primary-fixed z-10">
                {currentTotal}
              </div>
            )}
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={points} margin={{ top: 5, right: 5, bottom: 5, left: -20 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(125,211,252,0.06)" />
                <XAxis
                  dataKey="time" stroke="rgba(125,211,252,0.2)"
                  tick={{ fill: "#a0b4c4", fontSize: 9 }}
                  interval="preserveStartEnd"
                  minTickGap={50}
                />
                <YAxis
                  stroke="rgba(125,211,252,0.2)"
                  tick={{ fill: "#a0b4c4", fontSize: 9 }}
                />
                <Tooltip
                  contentStyle={{
                    background: "rgba(15, 21, 36, 0.9)",
                    border: "1px solid rgba(125, 211, 252, 0.2)",
                    borderRadius: "8px",
                    color: "#e0e8f0",
                    fontSize: "12px",
                  }}
                />
                <Line
                  type="stepAfter" dataKey="total" stroke="#7dd3fc" strokeWidth={2}
                  strokeDasharray="4 2" dot={false} isAnimationActive={false}
                  name="Total"
                />
                <Line
                  type="stepAfter" dataKey="inner" stroke="#c8eaff" strokeWidth={2}
                  dot={false} isAnimationActive={false}
                  filter="drop-shadow(0 0 4px rgba(200,234,255,0.5))"
                  name="Inner"
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>
      );
    }

    // ── Slam Rate Panel ──
    function SlamRatePanel() {
      const [rateData, setRateData] = useState({ msgs_per_sec: 0, status: "critical" });

      useEffect(() => {
        let alive = true;
        const poll = () => {
          fetch(`${API_BASE}/live/slam_rate`)
            .then(r => r.json())
            .then(d => { if (alive) setRateData(d); })
            .catch(() => {});
        };
        poll();
        const id = setInterval(poll, 500);
        return () => { alive = false; clearInterval(id); };
      }, []);

      const { msgs_per_sec, status } = rateData;
      const colorMap = { ok: "#4ade80", warn: "#facc15", critical: "#ff6b6b" };
      const color = colorMap[status] || colorMap.critical;

      return (
        <div className="glass-panel p-4 rounded-xl flex flex-col gap-3">
          <h3 className="font-label text-xs text-on-surface-variant mb-1 flex items-center gap-2">
            <span className="material-symbols-outlined text-sm">speed</span>
            SLAM Publish Rate
          </h3>

          <div
            className="flex items-center justify-between p-3 rounded-lg"
            style={{
              background: `${color}10`,
              border: `1px solid ${color}40`,
            }}
          >
            <span className="font-body text-sm text-on-surface-variant">msgs/sec</span>
            <span className="font-display text-2xl" style={{ color }}>
              {msgs_per_sec.toFixed(1)}
            </span>
          </div>

          {status === "warn" && (
            <div
              className="px-3 py-2 rounded text-sm font-body"
              style={{
                background: "rgba(250,204,21,0.12)",
                border: "1px solid rgba(250,204,21,0.35)",
                color: "#facc15",
              }}
            >
              ⚠ Low SLAM feature rate — degraded tracking
            </div>
          )}
          {status === "critical" && (
            <div
              className="px-3 py-2 rounded text-sm font-body"
              style={{
                background: "rgba(255,107,107,0.12)",
                border: "1px solid rgba(255,107,107,0.35)",
                color: "#ff6b6b",
              }}
            >
              ✖ SLAM rate critical — car may stop
            </div>
          )}
        </div>
      );
    }

    // ── Phase Timeline ──
    function PhaseTimeline({ historyData, livePhase }) {
      const [segments, setSegments] = useState([]);

      useEffect(() => {
        if (historyData.length > 0) {
          const segs = [];
          for (let i = 0; i < historyData.length; i++) {
            const row = historyData[i];
            const ts = typeof row.ts === "string" ? new Date(row.ts).getTime() / 1000 : row.ts;
            const nextTs = i + 1 < historyData.length
              ? (typeof historyData[i + 1].ts === "string"
                ? new Date(historyData[i + 1].ts).getTime() / 1000
                : historyData[i + 1].ts)
              : Date.now() / 1000;
            segs.push({
              phase: row.phase,
              start: ts,
              duration: nextTs - ts,
            });
          }
          setSegments(segs);
        }
      }, [historyData]);

      useEffect(() => {
        if (livePhase && livePhase.phase) {
          setSegments(prev => {
            const now = typeof livePhase.ts === "number" ? livePhase.ts : Date.now() / 1000;
            // Close last segment
            const updated = prev.map((s, i) => {
              if (i === prev.length - 1 && s.duration === 0) {
                return { ...s, duration: now - s.start };
              }
              return s;
            });
            // Only add if different from last phase
            const lastPhase = updated.length > 0 ? updated[updated.length - 1].phase : null;
            if (livePhase.phase !== lastPhase) {
            if (updated.length > 0) {
              const last = updated[updated.length - 1];
              updated[updated.length - 1] = { ...last, duration: now - last.start };
            }
              updated.push({ phase: livePhase.phase, start: now, duration: 0 });
            }
            return updated;
          });
        }
      }, [livePhase]);

      const totalDuration = segments.reduce((acc, s) => acc + Math.max(s.duration, 1), 0);

      return (
        <div id="phase-timeline" className="glass-panel p-4 rounded-xl">
          <h3 className="font-label text-xs text-on-surface-variant mb-3 flex items-center gap-2">
            <span className="material-symbols-outlined text-sm">timeline</span>
            Phase Timeline
          </h3>
          <div className="flex h-7 rounded-sm overflow-hidden border border-outline-variant">
            {segments.map((seg, i) => {
              const w = totalDuration > 0 ? (Math.max(seg.duration, 1) / totalDuration) * 100 : 0;
              const colors = PHASE_COLORS[seg.phase] || PHASE_COLORS.init;
              return (
                <div
                  key={i}
                  className="flex items-center justify-center overflow-hidden"
                  style={{
                    width: `${w}%`,
                    background: colors.bg,
                    borderRight: i < segments.length - 1 ? `1px solid ${colors.border}` : "none",
                    minWidth: "2px",
                  }}
                  title={`${seg.phase} — ${fmtDuration(seg.duration)}`}
                >
                  {w > 8 && (
                    <span className="text-[10px] px-1 truncate" style={{ color: colors.text }}>
                      {seg.phase.toUpperCase()}
                    </span>
                  )}
                </div>
              );
            })}
            {segments.length === 0 && (
              <div className="flex-1 flex items-center justify-center text-[10px] text-on-surface-variant">
                No phase data
              </div>
            )}
          </div>
          <div className="flex justify-between text-[10px] text-on-surface-variant mt-1">
            <span>-10m</span>
            <span>Now</span>
          </div>
        </div>
      );
    }

    // ── Toggle Switch ──
    function ToggleSwitch({ active, onToggle }) {
      return (
        <div
          className={`toggle-track ${active ? "active" : ""}`}
          onClick={onToggle}
          role="switch"
          aria-checked={active}
        >
          <div className="toggle-thumb" />
        </div>
      );
    }



    // ── Anomaly Injection Panel ──
    function AnomalyInjectionPanel({ toggles, onToggle, onImuDriftReset }) {
      const [simSpeed, setSimSpeed] = useState(1.0);
      const [motorStallSeverity, setMotorStallSeverity] = useState(0.5);
      const [slamQuality, setSlamQuality] = useState(50);

      const handleSimSpeedChange = (e) => {
        const val = parseFloat(e.target.value);
        setSimSpeed(val);
        fetch(`${API_BASE}/control/sim_speed`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ speed: val }),
        }).catch(() => {});
      };

      const handleMotorStallSeverityChange = (e) => {
        const val = parseFloat(e.target.value);
        setMotorStallSeverity(val);
        if (!toggles.motor_stall) return;
        fetch(`${API_BASE}/api/publish`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            topic: "car/mock/motor_stall_params",
            payload: { severity: parseFloat(val.toFixed(2)) },
          }),
        }).catch(() => {});
      };

      // Publish motor_stall severity on first activation
      const prevMotorRef = useRef(toggles.motor_stall);
      useEffect(() => {
        if (toggles.motor_stall && !prevMotorRef.current) {
          fetch(`${API_BASE}/api/publish`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              topic: "car/mock/motor_stall_params",
              payload: { severity: parseFloat(motorStallSeverity.toFixed(2)) },
            }),
          }).catch(() => {});
        }
        prevMotorRef.current = toggles.motor_stall;
      }, [toggles.motor_stall, motorStallSeverity]);

      const handleSlamQualityChange = (e) => {
        const val = parseInt(e.target.value, 10);
        setSlamQuality(val);
        if (!toggles.slam_low_feature) return;
        const decay = 0.980 + (val / 100) * (1.000 - 0.980);
        const recover = 1.005 + (val / 100) * (1.050 - 1.005);
        fetch(`${API_BASE}/api/publish`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            topic: "car/mock/slam_params",
            payload: { decay: parseFloat(decay.toFixed(4)), recover: parseFloat(recover.toFixed(4)) },
          }),
        }).catch(() => {});
      };

      // Publish slam params on first activation
      const prevSlamRef = useRef(toggles.slam_low_feature);
      useEffect(() => {
        if (toggles.slam_low_feature && !prevSlamRef.current) {
          const decay = 0.980 + (slamQuality / 100) * (1.000 - 0.980);
          const recover = 1.005 + (slamQuality / 100) * (1.050 - 1.005);
          fetch(`${API_BASE}/api/publish`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              topic: "car/mock/slam_params",
              payload: { decay: parseFloat(decay.toFixed(4)), recover: parseFloat(recover.toFixed(4)) },
            }),
          }).catch(() => {});
        }
        prevSlamRef.current = toggles.slam_low_feature;
      }, [toggles.slam_low_feature, slamQuality]);

      const renderSubControls = (key) => {
        switch (key) {
          case "motor_stall":
            return (
              <div className="flex flex-col gap-2 px-3 pb-3">
                <div className="flex items-center justify-between">
                  <span className="font-label text-xs text-on-surface-variant">Severity</span>
                  <span className="font-display text-sm" style={{ color: "#7dd3fc" }}>{motorStallSeverity.toFixed(2)}</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.01"
                  value={motorStallSeverity}
                  onChange={handleMotorStallSeverityChange}
                  className="w-full accent-cyan-400"
                  style={{ accentColor: "#7dd3fc" }}
                />
                <div className="flex justify-between text-xs font-body text-on-surface-variant opacity-70">
                  <span>0.0 (arc)</span>
                  <span>1.0 (freeze)</span>
                </div>
              </div>
            );
          case "slam_low_feature":
            return (
              <div className="flex flex-col gap-2 px-3 pb-3">
                <div className="flex items-center justify-between">
                  <span className="font-label text-xs text-on-surface-variant">Room Quality</span>
                  <span className="font-display text-sm" style={{ color: "#facc15" }}>{slamQuality}%</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="100"
                  value={slamQuality}
                  onChange={handleSlamQualityChange}
                  className="w-full accent-yellow-400"
                  style={{ accentColor: "#facc15" }}
                />
                <div className="flex justify-between text-xs font-body text-on-surface-variant opacity-70">
                  <span>0%</span>
                  <span>100%</span>
                </div>
              </div>
            );
          case "imu_static_drift":
            return (
              <div className="px-3 pb-3">
                <button
                  onClick={onImuDriftReset}
                  className="w-full px-4 py-2 rounded-lg font-label text-sm transition-colors"
                  style={{
                    background: "rgba(125,211,252,0.12)",
                    border: "1px solid rgba(125,211,252,0.3)",
                    color: "#7dd3fc",
                  }}
                  onMouseEnter={e => e.target.style.background = "rgba(125,211,252,0.2)"}
                  onMouseLeave={e => e.target.style.background = "rgba(125,211,252,0.12)"}
                >
                  Reset IMU
                </button>
              </div>
            );
          default:
            return null;
        }
      };

      return (
        <aside
          id="anomaly-injection-panel"
          className="w-80 border-l border-outline-variant p-gutter flex flex-col gap-6"
          style={{ background: "rgba(32, 44, 66, 0.4)" }}
        >
          <div>
            <h3 className="font-label text-sm text-on-surface uppercase tracking-widest flex items-center gap-2 mb-2">
              <span className="material-symbols-outlined text-sm text-secondary">speed</span>
              Simulation Speed
            </h3>
            <p className="font-label text-xs text-on-surface-variant mb-2">
              {simSpeed.toFixed(2)}×
            </p>
            <input
              type="range"
              min="0.25"
              max="4.0"
              step="0.25"
              value={simSpeed}
              onChange={handleSimSpeedChange}
              className="w-full accent-cyan-400"
              style={{ accentColor: "#7dd3fc" }}
            />
            <div className="flex justify-between text-xs font-body text-on-surface-variant opacity-70 mb-4">
              <span>0.25×</span>
              <span>4.0×</span>
            </div>
          </div>

          <div>
            <h3 className="font-label text-sm text-on-surface uppercase tracking-widest flex items-center gap-2 mb-2">
              <span className="material-symbols-outlined text-sm text-secondary">bug_report</span>
              Anomaly Injection
            </h3>
            <p className="font-label text-xs text-on-surface-variant mb-4">
              Trigger faults to test fallbacks.
            </p>
            <div className="space-y-3">
              {ANOMALY_TYPES.map(({ key, label, color }) => (
                <div
                  key={key}
                  className="glass-panel rounded-lg overflow-hidden"
                  style={{
                    border: `1px solid ${toggles[key] ? `${color}40` : "rgba(255,255,255,0.08)"}`,
                  }}
                >
                  <label
                    className="flex items-center justify-between p-3 cursor-pointer transition-colors"
                    style={{
                      ...(toggles[key] ? {
                        boxShadow: `0 0 12px ${color}26`,
                      } : {}),
                    }}
                    onMouseEnter={(e) => {
                      e.currentTarget.style.background = `${color}0d`;
                    }}
                    onMouseLeave={(e) => {
                      e.currentTarget.style.background = "";
                    }}
                  >
                    <span className="font-body text-sm text-on-surface">{label}</span>
                    <ToggleSwitch active={toggles[key]} onToggle={() => onToggle(key)} />
                  </label>
                  {toggles[key] && renderSubControls(key)}
                </div>
              ))}
            </div>
          </div>
        </aside>
      );
    }

    // ── Navigation Map Canvas ──
    function MapCanvas({ snapshot }) {
      const canvasRef = useRef(null);
      const CAR_R = 10, ARROW_LEN = 22;

      useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext("2d");

        const W = canvas.offsetWidth;
        const H = canvas.offsetHeight;
        canvas.width  = W;
        canvas.height = H;

        const PAD = 40;
        const X_MIN = -0.8,  X_MAX = 1.1;   // shift up to center start vertically
        const Z_MIN = -0.5,  Z_MAX = 1.8;   // keep Z range
        const scaleX = (W - PAD * 2) / (X_MAX - X_MIN);
        const scaleZ = (H - PAD * 2) / (Z_MAX - Z_MIN);
        const scale  = Math.min(scaleX, scaleZ);
        const wx = (x) => PAD + (x - X_MIN) * scale;
        const wz = (z) => H - PAD - (z - Z_MIN) * scale;

        // Background
        ctx.clearRect(0, 0, W, H);
        ctx.fillStyle = "#0d1117";
        ctx.fillRect(0, 0, W, H);

        // Grid
        ctx.strokeStyle = "rgba(255,255,255,0.06)";
        ctx.lineWidth = 1;
        ctx.setLineDash([]);

        // Vertical lines — iterate over X world values
        for (let v = -0.8; v <= 1.1; v = Math.round((v + 0.25) * 1000) / 1000) {
          const px = wx(v);
          ctx.beginPath();
          ctx.moveTo(px, 0);
          ctx.lineTo(px, H);
          ctx.stroke();
        }

        // Horizontal lines — iterate over Z world values
        for (let v = -0.5; v <= 1.8; v = Math.round((v + 0.25) * 1000) / 1000) {
          const pz = wz(v);
          ctx.beginPath();
          ctx.moveTo(0, pz);
          ctx.lineTo(W, pz);
          ctx.stroke();
        }

        // Stations as rectangles
        const RW = 18, RH = 10;
        ctx.textAlign = "center";
        for (const [name, s] of Object.entries(STATIONS)) {
          if (name.startsWith("_") || typeof s.x !== "number") continue;
          const px = wx(s.x), pz = wz(s.z);
          ctx.fillStyle = "#1f6feb";
          ctx.strokeStyle = "#e6edf3";
          ctx.lineWidth = 1.5;
          ctx.fillRect(px - RW / 2, pz - RH / 2, RW, RH);
          ctx.strokeRect(px - RW / 2, pz - RH / 2, RW, RH);
          ctx.fillStyle = "#8b949e";
          ctx.font = "11px monospace";
          ctx.textAlign = "center";
          ctx.fillText(name.replace('_', ' '), px, pz + RH / 2 + 14);
        }

        const slam  = snapshot["car/slam/pose"];
        const imu   = snapshot["car/imu"];
        const phase = snapshot["car/nav/phase"];

        // Car
        if (slam && slam.x != null && slam.z != null) {
          const cx = wx(slam.x), cz = wz(slam.z);
          const hdgRad = ((imu?.heading_deg ?? 0) * Math.PI / 180) - Math.PI / 2;

          ctx.beginPath();
          ctx.arc(cx, cz, CAR_R, 0, Math.PI * 2);
          ctx.fillStyle = "#e6edf3";
          ctx.fill();
          ctx.strokeStyle = "#30363d";
          ctx.lineWidth = 1.5;
          ctx.stroke();

          // Arrow
          const ax = cx + Math.cos(hdgRad) * ARROW_LEN;
          const az = cz + Math.sin(hdgRad) * ARROW_LEN;
          ctx.beginPath();
          ctx.moveTo(cx, cz);
          ctx.lineTo(ax, az);
          ctx.strokeStyle = "#238636";
          ctx.lineWidth = 2.5;
          ctx.lineCap = "round";
          ctx.stroke();
          ctx.lineCap = "butt";

          // Label
          ctx.fillStyle = "#8b949e";
          ctx.font = "500 10px system-ui";
          ctx.textAlign = "center";
          ctx.fillText("car", cx, cz + CAR_R + 12);
        }
      }, [snapshot]);

      return (
        <div id="nav-map" className="lg:col-span-8 glass-panel rounded-xl overflow-hidden flex flex-col relative min-h-[400px]">
          <div className="p-4 border-b border-outline-variant flex justify-between items-center z-10"
               style={{ background: "rgba(32, 44, 66, 0.3)" }}>
            <h3 className="font-headline text-lg text-primary glow-text-primary">Navigation Map</h3>
            <span className="font-label text-xs text-on-surface-variant">Live</span>
          </div>
          <div className="flex-1 relative z-[1] min-h-[340px] flex items-center justify-center"
               style={{ background: "#0a0e1a" }}>
            <div style={{ width: '100%', height: '500px', position: 'relative' }}>
              <canvas
                ref={canvasRef}
                style={{ width: '100%', height: '100%', display: 'block' }}
              />
            </div>
          </div>
          <div style={{ display: 'flex', gap: '8px', marginTop: '12px', justifyContent: 'center' }}>
            {Object.keys(STATIONS).map(name => (
              <button
                key={name}
                onClick={() => fetch('/api/goto', {
                  method: 'POST',
                  headers: { 'Content-Type': 'application/json' },
                  body: JSON.stringify({ target: name })
                })}
                style={{
                  background: 'rgba(255,255,255,0.05)',
                  border: '1px solid rgba(255,255,255,0.12)',
                  borderRadius: '8px',
                  color: '#e6edf3',
                  padding: '6px 16px',
                  fontSize: '13px',
                  cursor: 'pointer',
                  fontFamily: 'inherit',
                  letterSpacing: '0.03em',
                  transition: 'background 0.15s',
                }}
                onMouseEnter={e => e.target.style.background = 'rgba(255,255,255,0.10)'}
                onMouseLeave={e => e.target.style.background = 'rgba(255,255,255,0.05)'}
              >
                {name.replace('_', ' ')}
              </button>
            ))}
          </div>
        </div>
      );
    }

    // ─────────────────────────────────────────────
    // MAIN APP
    // ─────────────────────────────────────────────
    function App() {
      const { connected, snapshot } = useWebSocket();

      const liveSlam   = snapshot["car/slam/pose"] || null;
      const liveImu    = snapshot["car/imu"] || null;
      const liveMotors = snapshot["car/motors"] || null;
      const livePhase  = snapshot["car/nav/phase"] || null;

      const slamSeq = liveSlam?.seq ?? null;

      const [slamHistory]   = useHistory("/history/slam?minutes=10");
      const [imuHistory]    = useHistory("/history/imu?minutes=10");
      const [motorHistory]  = useHistory("/history/motors?minutes=10");
      const [phaseHistory]  = useHistory("/history/phase?minutes=10");

      // SLAM rate status (for path chart color)
      const [slamRateStatus, setSlamRateStatus] = useState("ok");
      useEffect(() => {
        let alive = true;
        const poll = () => {
          fetch(`${API_BASE}/live/slam_rate`)
            .then(r => r.json())
            .then(d => { if (alive) setSlamRateStatus(d.status); })
            .catch(() => {});
        };
        poll();
        const id = setInterval(poll, 500);
        return () => { alive = false; clearInterval(id); };
      }, []);

      // Anomaly toggle state
      const [toggles, setToggles] = useState(
        Object.fromEntries(ANOMALY_TYPES.map(a => [a.key, false]))
      );

      const handleToggle = useCallback(async (key) => {
        const newVal = !toggles[key];
        setToggles(prev => ({ ...prev, [key]: newVal }));
        try {
          await fetch(`${API_BASE}/api/inject`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ anomaly: key, active: newVal }),
          });
        } catch (e) {
          console.error("Inject failed:", e);
          setToggles(prev => ({ ...prev, [key]: !newVal }));
        }
      }, [toggles]);

      const handleImuDriftReset = useCallback(() => {
        fetch(`${API_BASE}/control/imu_drift_reset`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
        }).catch(() => {});
      }, []);

      return (
        <React.Fragment>
          <nav
            className="fixed top-0 w-full z-50 border-b border-outline-variant"
            style={{
              background: "rgba(10, 14, 26, 0.8)",
              backdropFilter: "blur(24px)",
              WebkitBackdropFilter: "blur(24px)",
              boxShadow: "0 20px 50px rgba(0,0,0,0.5)",
            }}
          >
            <div className="flex justify-between items-center px-gutter py-4">
              <div className="font-headline text-2xl font-bold tracking-tighter text-primary glow-text-primary">
                AeroMonitor v4.0
              </div>
              <div className="hidden md:flex space-x-8">
                <a className="text-on-surface-variant hover:text-primary transition-colors duration-300" href="#">Telemetry</a>
                <a className="text-primary border-b-2 border-primary pb-1 transition-colors duration-300 glow-text-primary" href="#">Navigation</a>
                <a className="text-on-surface-variant hover:text-primary transition-colors duration-300" href="#">Systems</a>
                <a className="text-on-surface-variant hover:text-primary transition-colors duration-300" href="#">Diagnostics</a>
              </div>
              <div className="flex items-center space-x-4">
                <button className="text-on-surface-variant hover:text-primary transition-colors duration-300 active:scale-95">
                  <span className="material-symbols-outlined">notifications</span>
                </button>
                <button className="text-on-surface-variant hover:text-primary transition-colors duration-300 active:scale-95">
                  <span className="material-symbols-outlined">settings</span>
                </button>
                <div
                  className="w-8 h-8 rounded-full overflow-hidden glow-border-primary flex items-center justify-center"
                  style={{ border: "1px solid rgba(125, 211, 252, 0.3)", background: "rgba(125, 211, 252, 0.1)" }}
                >
                  <span className="material-symbols-outlined text-primary text-sm">person</span>
                </div>
              </div>
            </div>
          </nav>

          <div className="flex-1 flex pt-20">
            <main className="flex-1 flex flex-col p-gutter gap-panel-gap min-h-[calc(100vh-5rem)]">
              <StatusBar
                slam={liveSlam}
                phase={livePhase}
                motors={liveMotors}
                slamSeq={slamSeq}
                wsConnected={connected}
              />

              <div className="grid grid-cols-1 lg:grid-cols-12 gap-panel-gap flex-1">
                <div className="lg:col-span-8 flex flex-col gap-panel-gap">
                  <MapCanvas snapshot={snapshot} />
                </div>

                {/* Right column: Phase + IMU + Motor + SLAM Rate */}
                <div className="lg:col-span-4 flex flex-col gap-panel-gap">
                  <PhaseTimeline historyData={phaseHistory} livePhase={livePhase} />
                  <IMUHeadingChart historyData={imuHistory} liveImu={liveImu} />
                  <MotorPWMChart historyData={motorHistory} liveMotors={liveMotors} />
                  <SlamRatePanel />
                </div>
              </div>
            </main>

            <AnomalyInjectionPanel
              toggles={toggles}
              onToggle={handleToggle}
              onImuDriftReset={handleImuDriftReset}
            />
          </div>
        </React.Fragment>
      );
    }

    // ─────────────────────────────────────────────
    // MOUNT
    // ─────────────────────────────────────────────
    const root = ReactDOM.createRoot(document.getElementById("root"));
    root.render(<App />);