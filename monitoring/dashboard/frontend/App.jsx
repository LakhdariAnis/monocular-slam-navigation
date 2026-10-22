// ─────────────────────────────────────────────
    // IMPORTS from global UMD bundles
    // ─────────────────────────────────────────────
    const {
      useState, useEffect, useRef, useCallback,
    } = React;

    const {
      ResponsiveContainer, AreaChart, Area,
      XAxis, YAxis, CartesianGrid, Tooltip, ReferenceLine,
    } = Recharts;

    // ─────────────────────────────────────────────
    // CONSTANTS
    // ─────────────────────────────────────────────
    // Same origin when served by the backend (works from a phone on the LAN); localhost when opened as a file.
    const API_BASE = location.protocol === "file:" ? "http://localhost:8000" : "";
    const WS_URL   = location.protocol === "file:"
      ? "ws://localhost:8000/ws"
      : `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`;

    const MAX_PATH_POINTS = 500;
    const PATH_MIN_STEP_M = 0.005;  // skip near-duplicate poses so the 500 points cover the drive, not 16 s of standing still
    const TRAIL_BREAK_M = 0.25;
    const MAX_CHART_POINTS = 600;
    const CHART_WINDOW_S = 120;
    const PHASE_WINDOW_S = 600;

    const C = {
      text: "#0f172a", dim: "#64748b", line: "#e5e7eb",
      ind: "#4f46e5", indSoft: "#a5b4fc", grn: "#10b981", amb: "#f59e0b", red: "#ef4444", vio: "#8b5cf6", grey: "#94a3b8",
    };

    const ANOMALY_TYPES = [
      { key: "tracking_loss",    label: "Tracking loss" },
      { key: "motor_stall",      label: "Motor stall" },
      { key: "position_jump",    label: "Position jump" },
      { key: "imu_static_drift", label: "IMU static drift" },
      { key: "slam_low_feature", label: "Low feature room" },
    ];

    const PHASE_COLORS = {
      departing: "#cbd5e1", phase1: "#818cf8", phase2: "#6366f1", phase3: "#a78bfa",
      aligning: "#c4b5fd", arrived: "#34d399", tracking_lost: "#ef4444", init: "#e2e8f0",
    };
    const NAV_PHASES = new Set(["departing", "phase1", "phase2", "phase3", "aligning", "tracking_lost"]);

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
    function toSec(ts) {
      return typeof ts === "number" ? ts : Date.parse(ts) / 1000;
    }

    function fmtTime(ts) {
      if (!ts) return "";
      const d = new Date(typeof ts === "number" ? ts * 1000 : ts);
      if (isNaN(d.getTime())) return "";
      return d.toLocaleTimeString("en-GB", { hour12: false });
    }

    // IMU heading may arrive as -180..180; charts and readouts use 0..360
    const norm360 = (deg) => ((deg % 360) + 360) % 360;

    function fmtDuration(seconds) {
      if (seconds < 60) return `${Math.round(seconds)}s`;
      const m = Math.floor(seconds / 60);
      const s = Math.round(seconds % 60);
      return `${m}m ${s}s`;
    }

    function post(path, body) {
      return fetch(`${API_BASE}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: body ? JSON.stringify(body) : undefined,
      });
    }

    const stationLabel = (name) => name === "start" ? "Start" : (name || "").replace("station_", "Station ");
    const typeLabel = (type) => {
      const known = ANOMALY_TYPES.find(a => a.key === type);
      return known ? known.label : (type || "").replace(/_/g, " ");
    };
    const phaseLabel = (phase) => phase ? phase.replace(/^phase(\d)$/, "Phase $1").replace(/_/g, " ").replace(/^./, c => c.toUpperCase()) : "—";

    function anomalyDetail(e) {
      if (e.severity === "CLEARED") return "resolved";
      switch (e.type) {
        case "motor_stall":
          return e.position_spread_mm != null
            ? `W held ${(Number(e.w_true_ratio) * 100).toFixed(0)}% · moved ${Number(e.position_spread_mm).toFixed(2)} mm`
            : `spinning · freeze streak ${e.freeze_streak}`;
        case "tracking_loss":
          return `${e.false_count}/${e.total_count} poses lost (${e.false_ratio})`;
        case "imu_static_drift":
          return `drift ${e.rate_first_half} → ${e.rate_second_half} °/s while still`;
        case "slam_low_feature":
          return `${e.rate_hz} Hz (${e.msg_count} msgs)`;
        default:
          return Object.entries(e).filter(([k]) => !["type", "severity", "ts"].includes(k)).map(([k, v]) => `${k} ${v}`).join(" · ");
      }
    }

    // ─────────────────────────────────────────────
    // CUSTOM HOOKS
    // ─────────────────────────────────────────────

    function useWebSocket(onAnomaly) {
      const [connected, setConnected] = useState(false);
      const [snapshot, setSnapshot]   = useState({});
      const wsRef  = useRef(null);
      const retry  = useRef(null);
      const onAnomalyRef = useRef(onAnomaly);
      onAnomalyRef.current = onAnomaly;

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
            // anomaly pushes are not snapshots; replacing the snapshot with them blanked the map for a frame
            if (data._anomaly) onAnomalyRef.current(data.entry);
            else setSnapshot(data);
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

    function usePoll(path, ms, initial) {
      const [data, setData] = useState(initial);
      useEffect(() => {
        let alive = true;
        const poll = () => {
          fetch(`${API_BASE}${path}`)
            .then(r => r.json())
            .then(d => { if (alive) setData(d); })
            .catch(() => {});
        };
        poll();
        const id = setInterval(poll, ms);
        return () => { alive = false; clearInterval(id); };
      }, [path, ms]);
      return data;
    }

    // Anomaly feed keyed by type: polled from /anomalies, plus instant WS pushes.
    function useAnomalies() {
      const [anomalies, setAnomalies] = useState({});
      const add = useCallback((entry) => {
        if (entry && entry.type != null) setAnomalies(prev => ({ ...prev, [entry.type]: entry }));
      }, []);

      useEffect(() => {
        let alive = true;
        const poll = () => {
          fetch(`${API_BASE}/anomalies`)
            .then(r => r.json())
            .then(data => {
              if (!alive || !Array.isArray(data)) return;
              setAnomalies(prev => {
                const next = { ...prev };
                for (const entry of data) {
                  if (entry.type != null) next[entry.type] = entry;
                }
                return next;
              });
            })
            .catch(() => {});
        };
        poll();
        const id = setInterval(poll, 3000);
        return () => { alive = false; clearInterval(id); };
      }, []);

      const clear = useCallback(() => {
        post("/anomalies/clear").catch(() => {});
        setAnomalies({});
      }, []);

      return { anomalies, add, clear };
    }

    // Time series = history preload + live points, thinned to one point per minDt seconds.
    function useSeries(historyData, live, toPoint, minDt) {
      const [points, setPoints] = useState([]);

      useEffect(() => {
        if (historyData.length === 0) return;
        const out = [];
        for (const row of historyData) {
          const p = toPoint(row);
          if (!p) continue;
          if (out.length && p.t - out[out.length - 1].t < minDt) continue;
          out.push(p);
        }
        setPoints(prev => [...out, ...prev.filter(p => !out.length || p.t > out[out.length - 1].t)].slice(-MAX_CHART_POINTS));
      }, [historyData]);

      useEffect(() => {
        const p = live && toPoint(live);
        if (!p) return;
        setPoints(prev => {
          const last = prev[prev.length - 1];
          if (last && p.t - last.t < Math.max(minDt, 1e-6)) return prev;
          return [...prev, p].slice(-MAX_CHART_POINTS);
        });
      }, [live?.ts]);

      return points;
    }

    // Driven trail (last 500 distinct poses) + last known good pose for the LOST state.
    function usePath(historyData, slam) {
      const [path, setPath] = useState([]);
      const lastGood = useRef(null);

      const append = (arr, x, z) => {
        const last = arr[arr.length - 1];
        if (!last || Math.hypot(last[0] - x, last[1] - z) >= PATH_MIN_STEP_M) arr.push([x, z]);
      };

      useEffect(() => {
        if (historyData.length === 0) return;
        const pts = [];
        let good = null;
        for (const r of historyData) {
          if (r.ok === false || r.x == null || r.z == null) continue;
          append(pts, r.x, r.z);
          good = r;
        }
        if (good && !lastGood.current) lastGood.current = { x: good.x, z: good.z, ts: toSec(good.ts) };
        setPath(prev => {
          const merged = [...pts];
          for (const [x, z] of prev) append(merged, x, z);
          return merged.slice(-MAX_PATH_POINTS);
        });
      }, [historyData]);

      useEffect(() => {
        if (!slam || !slam.ok || slam.x == null || slam.z == null) return;
        lastGood.current = { x: slam.x, z: slam.z, ts: slam.ts };
        setPath(prev => {
          const next = [...prev];
          append(next, slam.x, slam.z);
          return next.length === prev.length ? prev : next.slice(-MAX_PATH_POINTS);
        });
      }, [slam?.seq, slam?.ts]);

      return { path, lastGood: lastGood.current };
    }

    // Phase segments for the timeline + the current leg (from = last station the car arrived at).
    function useNav(historyData, livePhase) {
      const [segments, setSegments] = useState([]);
      const [leg, setLeg] = useState(null);
      const docked = useRef("start");
      const legRef = useRef(null);

      const apply = (row, segs) => {
        const start = toSec(row.ts);
        const last = segs[segs.length - 1];
        if (!last || last.phase !== row.phase) segs.push({ phase: row.phase, target: row.target_station, start });
        if (row.phase === "arrived") {
          if (row.target_station) docked.current = row.target_station;
          legRef.current = null;
        } else if (NAV_PHASES.has(row.phase) && STATIONS[row.target_station]) {
          if (!legRef.current || legRef.current.to !== row.target_station) {
            legRef.current = { from: STATIONS[docked.current] ? docked.current : "start", to: row.target_station };
          }
        }
      };

      const segsRef = useRef([]);
      const commit = (segs) => { segsRef.current = segs; setSegments(segs); setLeg(legRef.current); };

      useEffect(() => {
        if (historyData.length === 0) return;
        // replay history, then any live segments that arrived first, so docked/leg end up in time order
        const live = segsRef.current;
        docked.current = "start"; legRef.current = null;
        const segs = [];
        for (const row of historyData) if (row.phase) apply(row, segs);
        const lastHist = segs.length ? segs[segs.length - 1].start : -Infinity;
        for (const s of live) if (s.start > lastHist) apply({ phase: s.phase, target_station: s.target, ts: s.start }, segs);
        commit(segs);
      }, [historyData]);

      useEffect(() => {
        if (!livePhase || !livePhase.phase) return;
        const segs = [...segsRef.current];
        apply({ ...livePhase, ts: livePhase.ts ?? Date.now() / 1000 }, segs);
        if (segs.length !== segsRef.current.length) commit(segs);
        else setLeg(legRef.current);
      }, [livePhase?.ts, livePhase?.phase]);

      const resetDocked = useCallback(() => { docked.current = "start"; legRef.current = null; setLeg(null); }, []);

      return { segments, leg, resetDocked };
    }

    // ─────────────────────────────────────────────
    // MAP RENDERING (canvas)
    // ─────────────────────────────────────────────
    function niceStep(pxPerM) {
      for (const s of [0.05, 0.1, 0.25, 0.5, 1, 2, 5]) if (s * pxPerM >= 56) return s;
      return 10;
    }
    const fmtTick = (v) => (Math.abs(v) < 1e-9 ? "0" : v.toFixed(2).replace(/\.?0+$/, ""));
    const SANS = "Manrope, system-ui, sans-serif";
    const MONO = '"JetBrains Mono", ui-monospace, monospace';

    function pill(ctx, x, y, w, h) {
      ctx.beginPath();
      ctx.roundRect(x, y, w, h, h / 2);
    }

    // st: { path, pose, lastGood, heading, leg, phase, alert, now }
    function drawMap(ctx, W, H, st) {
      const compact = W < 520;
      const PADL = compact ? 36 : 46, PADB = 28, PADT = 12, PADR = 12;
      const pw = W - PADL - PADR, ph = H - PADT - PADB;
      if (pw <= 0 || ph <= 0) return;

      const tracking = st.pose && st.pose.ok && st.pose.x != null && st.pose.z != null;
      const p = tracking ? [st.pose.x, st.pose.z] : st.lastGood ? [st.lastGood.x, st.lastGood.z] : null;
      const lost = !tracking && !!p;

      // auto-fit bounds: stations + standoffs + path + car, with margin
      const xs = [], zs = [];
      for (const s of Object.values(STATIONS)) { xs.push(s.x, s.standoff[0]); zs.push(s.z, s.standoff[1]); }
      for (const [x, z] of st.path) { xs.push(x); zs.push(z); }
      if (p) { xs.push(p[0]); zs.push(p[1]); }
      const M = 0.3;
      const x0 = Math.min(...xs) - M, x1 = Math.max(...xs) + M, z0 = Math.min(...zs) - M, z1 = Math.max(...zs) + M;
      const scale = Math.min(pw / (x1 - x0), ph / (z1 - z0));
      const cxm = (x0 + x1) / 2, czm = (z0 + z1) / 2;
      const X = (x) => PADL + pw / 2 + (x - cxm) * scale;
      const Z = (z) => PADT + ph / 2 - (z - czm) * scale;
      const vx0 = cxm - pw / 2 / scale, vx1 = cxm + pw / 2 / scale;
      const vz0 = czm - ph / 2 / scale, vz1 = czm + ph / 2 / scale;

      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#ffffff"; ctx.fillRect(0, 0, W, H);
      ctx.save();
      ctx.beginPath(); ctx.rect(PADL, PADT, pw, ph); ctx.clip();
      ctx.fillStyle = "#fafbfd"; ctx.fillRect(PADL, PADT, pw, ph);

      // grid over the whole plot (minor = half step), zero axes highlighted
      const step = niceStep(scale), minor = step / 2;
      const gridLine = (v, vertical) => {
        const zero = Math.abs(v) < 1e-9;
        const major = Math.abs(v / step - Math.round(v / step)) < 1e-6;
        ctx.strokeStyle = zero ? "#c7d2fe" : major ? "#e3e7ee" : "#f1f3f7";
        ctx.lineWidth = zero ? 1.5 : 1;
        ctx.beginPath();
        if (vertical) { const x = Math.round(X(v)) + 0.5; ctx.moveTo(x, PADT); ctx.lineTo(x, PADT + ph); }
        else { const z = Math.round(Z(v)) + 0.5; ctx.moveTo(PADL, z); ctx.lineTo(PADL + pw, z); }
        ctx.stroke();
      };
      for (let v = Math.ceil(vx0 / minor) * minor; v <= vx1; v += minor) gridLine(+v.toFixed(6), true);
      for (let v = Math.ceil(vz0 / minor) * minor; v <= vz1; v += minor) gridLine(+v.toFixed(6), false);

      // walls behind each station + dotted standoff link
      for (const s of Object.values(STATIONS)) {
        const sign = s.orientation[0] === "+" ? 1 : -1, alongZ = s.orientation[1] === "X";
        const L = 0.24, D = 0.05;
        const [ax, az, bx, bz] = alongZ ? [s.x, s.z - L, s.x + sign * D, s.z + L] : [s.x - L, s.z, s.x + L, s.z + sign * D];
        const rx = Math.min(X(ax), X(bx)), ry = Math.min(Z(az), Z(bz));
        const rw = Math.abs(X(bx) - X(ax)), rh = Math.abs(Z(bz) - Z(az));
        ctx.fillStyle = "#e2e8f0"; ctx.fillRect(rx, ry, rw, rh);
        ctx.strokeStyle = C.grey; ctx.lineWidth = 1; ctx.strokeRect(rx, ry, rw, rh);
        ctx.setLineDash([2, 3]); ctx.strokeStyle = "#cbd5e1";
        ctx.beginPath(); ctx.moveTo(X(s.standoff[0]), Z(s.standoff[1])); ctx.lineTo(X(s.x), Z(s.z)); ctx.stroke();
        ctx.setLineDash([]);
      }

      // planned L-path from the car: phase1 -> elbow -> standoff, phase2 -> standoff, then standoff -> station
      if (st.leg && p && NAV_PHASES.has(st.phase)) {
        const { elbow, standoff, target } = computeElbow(st.leg.from, st.leg.to);
        const pts = [p];
        if (st.phase === "departing" || st.phase === "phase1") pts.push(elbow);
        if (st.phase !== "phase3" && st.phase !== "aligning") pts.push(standoff);
        ctx.strokeStyle = C.amb; ctx.lineWidth = 2.5; ctx.lineCap = "round";
        ctx.setLineDash([6, 6]);
        ctx.beginPath(); ctx.moveTo(X(pts[0][0]), Z(pts[0][1]));
        for (const q of pts.slice(1)) ctx.lineTo(X(q[0]), Z(q[1]));
        ctx.stroke();
        const tail = pts[pts.length - 1];
        ctx.setLineDash([1, 4]);
        ctx.beginPath(); ctx.moveTo(X(tail[0]), Z(tail[1])); ctx.lineTo(X(target[0]), Z(target[1])); ctx.stroke();
        ctx.setLineDash([]);
        if (pts.length > 2) { ctx.fillStyle = C.amb; ctx.beginPath(); ctx.arc(X(elbow[0]), Z(elbow[1]), 3, 0, Math.PI * 2); ctx.fill(); }
      }

      // driven trail, older = fainter
      const n = st.path.length;
      ctx.lineWidth = 3; ctx.lineCap = "round";
      for (let i = 1; i < n; i++) {
        // break the line at teleports (sim reset, position jumps) instead of drawing a chord across the map
        if (Math.hypot(st.path[i][0] - st.path[i - 1][0], st.path[i][1] - st.path[i - 1][1]) > TRAIL_BREAK_M) continue;
        ctx.strokeStyle = `rgba(79,70,229,${0.05 + 0.9 * (i / n)})`;
        ctx.beginPath();
        ctx.moveTo(X(st.path[i - 1][0]), Z(st.path[i - 1][1]));
        ctx.lineTo(X(st.path[i][0]), Z(st.path[i][1]));
        ctx.stroke();
      }

      // standoff points, station markers, labels
      for (const [name, s] of Object.entries(STATIONS)) {
        const target = st.leg && st.leg.to === name && NAV_PHASES.has(st.phase);
        const col = name === "start" ? C.grn : C.ind;
        ctx.beginPath(); ctx.arc(X(s.standoff[0]), Z(s.standoff[1]), 4, 0, Math.PI * 2);
        ctx.fillStyle = "#fafbfd"; ctx.fill(); ctx.strokeStyle = C.grey; ctx.lineWidth = 1.5; ctx.stroke();

        const sx = X(s.x), sz = Z(s.z);
        if (target) { ctx.beginPath(); ctx.arc(sx, sz, 16, 0, Math.PI * 2); ctx.fillStyle = "rgba(79,70,229,0.12)"; ctx.fill(); }
        ctx.beginPath(); ctx.arc(sx, sz, 8, 0, Math.PI * 2);
        ctx.fillStyle = target ? col : "#fff"; ctx.fill();
        ctx.strokeStyle = col; ctx.lineWidth = 3; ctx.stroke();

        // label past the end of the wall, away from the approach path
        const alongZ = s.orientation[1] === "X";
        const t = stationLabel(name), coords = `${s.x.toFixed(2)}, ${s.z.toFixed(2)}`;
        ctx.font = `700 12px ${SANS}`; const w1 = ctx.measureText(t).width;
        ctx.font = `500 10px ${MONO}`; const w2 = compact ? 0 : ctx.measureText(coords).width;
        const w = w1 + w2 + (compact ? 30 : 34), hh = 24;
        const lx = alongZ ? sx - w / 2 : X(s.x + 0.24) + 10;
        const lz = alongZ ? Z(s.z - 0.24) + 10 : sz - hh / 2;
        ctx.save();
        ctx.shadowColor = "rgba(16,24,40,0.15)"; ctx.shadowBlur = 8; ctx.shadowOffsetY = 2;
        pill(ctx, lx, lz, w, hh); ctx.fillStyle = "#fff"; ctx.fill();
        ctx.restore();
        if (target) { pill(ctx, lx, lz, w, hh); ctx.strokeStyle = C.ind; ctx.lineWidth = 1.5; ctx.stroke(); }
        ctx.beginPath(); ctx.arc(lx + 12, lz + hh / 2, 4, 0, Math.PI * 2); ctx.fillStyle = col; ctx.fill();
        ctx.textAlign = "left"; ctx.textBaseline = "middle";
        ctx.fillStyle = C.text; ctx.font = `700 12px ${SANS}`; ctx.fillText(t, lx + 22, lz + hh / 2 + 1);
        if (!compact) { ctx.fillStyle = C.dim; ctx.font = `500 10px ${MONO}`; ctx.fillText(coords, lx + 28 + w1, lz + hh / 2 + 1); }
      }

      // car: puck + heading arrow/cone; greyed at the last known position when tracking is lost
      if (p) {
        const x = X(p[0]), z = Z(p[1]);
        const hasHeading = st.heading != null;
        const a = ((st.heading ?? 0) * Math.PI) / 180;  // 0° = +Z (up), 90° = +X (right)
        const col = lost ? C.grey : st.alert ? C.red : C.ind;
        if (hasHeading) {
          ctx.beginPath(); ctx.moveTo(x, z); ctx.arc(x, z, 56, a - Math.PI / 2 - 0.35, a - Math.PI / 2 + 0.35); ctx.closePath();
          const g = ctx.createRadialGradient(x, z, 0, x, z, 56);
          g.addColorStop(0, lost ? "rgba(148,163,184,0.35)" : st.alert ? "rgba(239,68,68,0.35)" : "rgba(79,70,229,0.35)");
          g.addColorStop(1, "rgba(255,255,255,0)");
          ctx.fillStyle = g; ctx.fill();
        }
        if (st.alert && !lost) {
          for (const r of [22, 30]) {
            ctx.beginPath(); ctx.arc(x, z, r, 0, Math.PI * 2);
            ctx.strokeStyle = `rgba(239,68,68,${r === 22 ? 0.45 : 0.2})`; ctx.lineWidth = 2; ctx.stroke();
          }
        }
        ctx.save();
        ctx.shadowColor = "rgba(16,24,40,0.3)"; ctx.shadowBlur = 10; ctx.shadowOffsetY = 3;
        ctx.beginPath(); ctx.arc(x, z, 13, 0, Math.PI * 2); ctx.fillStyle = col; ctx.fill();
        ctx.restore();
        ctx.setLineDash(lost ? [3, 3] : []);
        ctx.beginPath(); ctx.arc(x, z, 13, 0, Math.PI * 2); ctx.strokeStyle = "#fff"; ctx.lineWidth = 3; ctx.stroke();
        ctx.setLineDash([]);
        if (hasHeading) {
          ctx.save(); ctx.translate(x, z); ctx.rotate(a);
          ctx.beginPath(); ctx.moveTo(0, -7); ctx.lineTo(5, 4); ctx.lineTo(0, 2); ctx.lineTo(-5, 4); ctx.closePath();
          ctx.fillStyle = "#fff"; ctx.fill();
          ctx.restore();
        }
        const lostFor = st.lastGood ? Math.max(0, st.now - st.lastGood.ts) : 0;
        const t = lost ? `LOST ${lostFor.toFixed(1)}s · last known` : st.alert ? "Stalled" : `${Math.round(norm360(st.heading ?? 0))}°`;
        ctx.font = `700 12px ${SANS}`;
        const w = ctx.measureText(t).width + 20;
        const tx = x - w - 24 < PADL + 4 ? x + 24 : x - w - 24;
        ctx.save();
        ctx.shadowColor = "rgba(16,24,40,0.15)"; ctx.shadowBlur = 8;
        pill(ctx, tx, z - 12, w, 24); ctx.fillStyle = lost || st.alert ? C.red : "#fff"; ctx.fill();
        ctx.restore();
        ctx.fillStyle = lost || st.alert ? "#fff" : C.text;
        ctx.textAlign = "left"; ctx.textBaseline = "middle";
        ctx.fillText(t, tx + 10, z + 1);
      }
      ctx.restore();

      // axes with metre ticks
      ctx.fillStyle = "#fff"; ctx.fillRect(0, 0, PADL, H); ctx.fillRect(0, PADT + ph, W, PADB);
      ctx.strokeStyle = C.line; ctx.lineWidth = 1; ctx.strokeRect(PADL + 0.5, PADT + 0.5, pw, ph);
      ctx.fillStyle = C.grey; ctx.font = `10px ${MONO}`;
      ctx.textAlign = "center"; ctx.textBaseline = "top";
      for (let v = Math.ceil(vx0 / step) * step; v <= vx1 + 1e-9; v += step) {
        if (X(v) < PADL + 10 || X(v) > PADL + pw - 30) continue;
        ctx.fillText(fmtTick(v), X(v), PADT + ph + 7);
      }
      ctx.textAlign = "right"; ctx.textBaseline = "middle";
      for (let v = Math.ceil(vz0 / step) * step; v <= vz1 + 1e-9; v += step) {
        if (Z(v) < PADT + 26 || Z(v) > PADT + ph - 8) continue;
        ctx.fillText(fmtTick(v), PADL - 6, Z(v));
      }
      ctx.fillStyle = C.text; ctx.font = `600 10px ${SANS}`;
      ctx.textAlign = "right"; ctx.textBaseline = "top"; ctx.fillText("X m", PADL + pw, PADT + ph + 7);
      ctx.textBaseline = "middle"; ctx.fillText("Z m", PADL - 6, PADT + 10);

      // scale bar
      const sbw = step * scale, sbx = PADL + pw - sbw - 16, sby = PADT + ph - 14;
      ctx.fillStyle = "rgba(255,255,255,0.9)"; ctx.fillRect(sbx - 8, sby - 20, sbw + 16, 30);
      ctx.strokeStyle = C.text; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(sbx, sby - 5); ctx.lineTo(sbx, sby); ctx.lineTo(sbx + sbw, sby); ctx.lineTo(sbx + sbw, sby - 5); ctx.stroke();
      ctx.fillStyle = C.text; ctx.font = `600 10px ${MONO}`; ctx.textAlign = "center"; ctx.textBaseline = "bottom";
      ctx.fillText(`${step} m`, sbx + sbw / 2, sby - 5);

      if (!p) {
        ctx.fillStyle = C.dim; ctx.font = `600 13px ${SANS}`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText("Waiting for SLAM pose…", PADL + pw / 2, PADT + 20);
      }
    }

    // ─────────────────────────────────────────────
    // COMPONENTS
    // ─────────────────────────────────────────────

    function Card({ className = "", children, ...rest }) {
      return <div className={`card ${className}`} {...rest}>{children}</div>;
    }

    // ── Header ──
    function Header({ wsConnected, slamSeq }) {
      return (
        <header className="flex items-center gap-3 flex-none">
          <div className="w-[30px] h-[30px] rounded-[9px] grid place-items-center text-white font-extrabold"
               style={{ background: "linear-gradient(135deg,#6366f1,#4f46e5)" }}>M</div>
          <div className="font-extrabold text-lg">Monitoring</div>
          <span className="k hidden sm:inline">Autonomous car · live</span>
          <div className="ml-auto flex items-center gap-2 text-[13px] font-semibold bg-white rounded-full px-3 py-1.5 shadow-sm">
            <span className="w-2 h-2 rounded-full"
                  style={{ background: wsConnected ? C.grn : C.red, boxShadow: `0 0 0 3px ${wsConnected ? "rgba(16,185,129,.2)" : "rgba(239,68,68,.2)"}` }} />
            {wsConnected ? "Connected" : "Disconnected"}
            {slamSeq != null && <span className="mono text-[11px]" style={{ color: C.dim }}>#{slamSeq}</span>}
          </div>
        </header>
      );
    }

    // ── Alert banner: every active (non-cleared) anomaly, CRIT first ──
    function AlertBanner({ anomalies, onClear }) {
      const active = Object.values(anomalies)
        .filter(e => e.severity !== "CLEARED")
        .sort((a, b) => (a.severity === "CRIT" ? 0 : 1) - (b.severity === "CRIT" ? 0 : 1));
      if (active.length === 0) return null;
      const crit = active[0].severity === "CRIT";
      const tone = crit
        ? { bg: "#fef2f2", border: "#fecaca", icon: C.red, title: "#991b1b", body: "#b91c1c" }
        : { bg: "#fffbeb", border: "#fde68a", icon: C.amb, title: "#92400e", body: "#b45309" };
      return (
        <div role="alert" className="flex-none flex items-center gap-3 rounded-[14px] px-3.5 py-2.5"
             style={{ background: tone.bg, border: `1px solid ${tone.border}` }}>
          <div className="ic text-white font-extrabold" style={{ background: tone.icon }}>!</div>
          <div className="min-w-0 flex flex-col gap-0.5">
            {active.map(e => (
              <div key={e.type} className="min-w-0">
                <span className="font-bold text-[15px]" style={{ color: tone.title }}>{typeLabel(e.type)}</span>
                <span className="mono text-[11px] font-semibold ml-2 px-1.5 py-0.5 rounded"
                      style={{ background: e.severity === "CRIT" ? C.red : C.amb, color: "#fff" }}>{e.severity}</span>
                <span className="text-[13px] ml-2" style={{ color: tone.body }}>{anomalyDetail(e)} · {fmtTime(e.ts)}</span>
              </div>
            ))}
          </div>
          <button onClick={onClear} className="ml-auto shrink-0 text-[13px] font-semibold bg-white rounded-[10px] px-3 py-1.5"
                  style={{ border: `1px solid ${tone.border}`, color: tone.title }}>
            Clear alerts
          </button>
        </div>
      );
    }

    // ── KPI row ──
    function Kpi({ icon, iconBg, iconColor, label, value, unit, badge, badgeBg, badgeColor, extra }) {
      return (
        <Card className="px-3.5 py-3 flex gap-3 items-center min-w-0">
          <div className="ic" style={{ background: iconBg, color: iconColor }}>{icon}</div>
          <div className="flex-1 min-w-0">
            <div className="k truncate">{label}</div>
            <div className="text-[22px] font-extrabold leading-tight truncate">
              {value}{unit && <span className="text-[13px] font-semibold ml-1" style={{ color: C.dim }}>{unit}</span>}
            </div>
          </div>
          {badge && (
            <span className="text-xs font-bold rounded-full px-2.5 py-0.5 whitespace-nowrap" style={{ background: badgeBg, color: badgeColor }}>{badge}</span>
          )}
          {extra}
        </Card>
      );
    }

    function KpiRow({ slam, slamRate, phase, phaseSince, motors, imu, now }) {
      const tracking = slam?.ok === true, lost = slam?.ok === false;
      const rateTone = { ok: [C.grn, "#d1fae5", "#047857"], warn: [C.amb, "#fef3c7", "#b45309"], critical: [C.red, "#fee2e2", "#b91c1c"] }[slamRate.status] || [C.grey, "#f1f5f9", C.dim];
      const slamBadge = lost ? ["LOST", "#fee2e2", "#b91c1c"] : tracking ? [slamRate.status === "ok" ? "OK" : slamRate.status.toUpperCase(), rateTone[1], rateTone[2]] : ["—", "#f1f5f9", C.dim];

      const total = motors?.total ?? 0, inner = motors?.inner ?? 0;
      const dir = total === 0 ? "stopped" : motors?.w ? "forward" : motors?.s ? "reverse" : motors?.a ? "spin left" : motors?.d ? "spin right" : "running";
      const ph = phase?.phase;

      return (
        <div className="flex-none grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3">
          <Kpi icon="◉" iconBg={lost ? "#fef2f2" : "#ecfdf5"} iconColor={lost ? C.red : C.grn}
               label={lost ? "SLAM tracking lost" : "SLAM tracking"} value={slamRate.msgs_per_sec.toFixed(1)} unit="msg/s"
               badge={slamBadge[0]} badgeBg={slamBadge[1]} badgeColor={slamBadge[2]} />
          <Kpi icon="➜" iconBg={ph === "tracking_lost" ? "#fef2f2" : "#eef2ff"} iconColor={ph === "tracking_lost" ? C.red : C.ind}
               label={phaseSince ? `Phase · ${fmtDuration(Math.max(0, now - phaseSince))}` : "Phase"} value={phaseLabel(ph)}
               badge={phase?.target_station ? (ph === "arrived" ? `at ${stationLabel(phase.target_station)}` : `to ${stationLabel(phase.target_station)}`) : null}
               badgeBg="#e0e7ff" badgeColor={C.ind} />
          <Kpi icon="⚙" iconBg="#fff7ed" iconColor={C.amb} label={`Motors · ${dir}`} value={total} unit={`/ ${inner} inner`}
               extra={
                 <div className="grid grid-cols-3 gap-0.5" aria-label="WASD keys">
                   <span />{["w"].map(k => <Key key={k} k={k} on={motors?.[k]} />)}<span />
                   {["a", "s", "d"].map(k => <Key key={k} k={k} on={motors?.[k]} />)}
                 </div>
               } />
          <Kpi icon="⬆" iconBg="#f5f3ff" iconColor={C.vio} label="Heading"
               value={imu?.heading_deg != null ? `${Math.round(norm360(imu.heading_deg))}°` : "—"}
               extra={slam?.x != null && <span className="mono text-[11px] whitespace-nowrap" style={{ color: C.dim }}>{slam.x.toFixed(2)}, {slam.z.toFixed(2)} m</span>} />
        </div>
      );
    }

    function Key({ k, on }) {
      return (
        <span className="w-[18px] h-[18px] rounded grid place-items-center mono text-[9px] font-bold uppercase"
              style={{ background: on ? C.ind : "#eef0f4", color: on ? "#fff" : C.grey }}>{k}</span>
      );
    }

    // ── Map card ──
    function MapCanvas(props) {
      const canvasRef = useRef(null);
      const propsRef = useRef(props);
      propsRef.current = props;

      // ResizeObserver + devicePixelRatio: canvas follows its box and stays sharp on HiDPI
      const draw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const r = canvas.getBoundingClientRect();
        if (!r.width || !r.height) return;
        const dpr = window.devicePixelRatio || 1;
        const w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
        if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
        const ctx = canvas.getContext("2d");
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        drawMap(ctx, r.width, r.height, propsRef.current);
      }, []);

      useEffect(() => {
        const ro = new ResizeObserver(draw);
        ro.observe(canvasRef.current);
        document.fonts?.ready.then(draw);
        return () => ro.disconnect();
      }, [draw]);
      useEffect(draw);

      return <canvas ref={canvasRef} className="block w-full h-full" aria-label="Navigation map" />;
    }

    function MapCard({ mapProps, currentTarget, onGoto, onReset, segments, now }) {
      return (
        <Card className="flex flex-col min-h-0 overflow-hidden h-[75vh] min-h-[420px] lg:h-auto lg:min-h-0">
          <div className="flex items-center gap-x-3 gap-y-2 px-3.5 pt-3 pb-2 flex-wrap">
            <div className="font-extrabold text-base">Map</div>
            <div className="flex gap-3 text-xs font-semibold" style={{ color: C.dim }}>
              <span><span className="inline-block w-3.5 h-[3px] rounded align-middle" style={{ background: C.ind }} /> Driven</span>
              <span><span className="inline-block w-3.5 align-middle" style={{ borderTop: `2.5px dashed ${C.amb}` }} /> Planned</span>
              <span>○ Standoff</span>
            </div>
            <div className="ml-auto flex gap-2 items-center flex-wrap">
              <div className="seg" role="group" aria-label="Go to station">
                {Object.keys(STATIONS).map(name => (
                  <button key={name} className={currentTarget === name ? "on" : ""} onClick={() => onGoto(name)}>
                    {stationLabel(name)}
                  </button>
                ))}
              </div>
              <button onClick={onReset} className="text-[13px] font-semibold rounded-[9px] px-2.5 py-1.5"
                      style={{ color: C.dim, border: `1px solid ${C.line}` }}>
                ↺ Reset to start
              </button>
            </div>
          </div>
          <div className="flex-1 min-h-0 mx-2"><MapCanvas {...mapProps} /></div>
          <PhaseTimeline segments={segments} now={now} />
        </Card>
      );
    }

    // ── Phase timeline (last 10 min) ──
    function PhaseTimeline({ segments, now }) {
      const from = now - PHASE_WINDOW_S;
      const bars = [];
      segments.forEach((s, i) => {
        const end = i + 1 < segments.length ? segments[i + 1].start : now;
        const a = Math.max(s.start, from);
        if (end <= a) return;
        bars.push({ ...s, dur: end - a, current: i === segments.length - 1 });
      });
      const total = bars.reduce((acc, b) => acc + b.dur, 0);
      const shown = Object.keys(PHASE_COLORS).filter(k => k !== "init");
      return (
        <div className="px-3.5 pt-2 pb-3">
          <div className="flex justify-between text-xs font-semibold mb-1.5" style={{ color: C.dim }}>
            <span>Phase history</span><span>last 10 min</span>
          </div>
          <div className="flex gap-[3px] h-2.5">
            {bars.map((b, i) => (
              <div key={i} title={`${b.phase} · ${fmtDuration(b.dur)}`}
                   style={{
                     width: `${(b.dur / total) * 100}%`, minWidth: 3, borderRadius: 5,
                     background: PHASE_COLORS[b.phase] || PHASE_COLORS.init,
                     boxShadow: b.current ? `0 0 0 2px #fff, 0 0 0 4px ${PHASE_COLORS[b.phase] || C.grey}` : "none",
                   }} />
            ))}
            {bars.length === 0 && <div className="flex-1 rounded bg-slate-100" />}
          </div>
          <div className="flex flex-wrap gap-x-3.5 gap-y-1 mt-2 text-[11px] font-semibold" style={{ color: C.dim }}>
            {shown.map(k => (
              <span key={k}><span className="inline-block w-2 h-2 rounded-[3px]" style={{ background: PHASE_COLORS[k] }} /> {k.replace("_", " ")}</span>
            ))}
          </div>
        </div>
      );
    }

    // ── Alerts card (anomaly feed) ──
    function AlertsCard({ anomalies, onClear }) {
      const entries = Object.values(anomalies).sort((a, b) => {
        const rank = (e) => e.severity === "CRIT" ? 0 : e.severity === "WARN" ? 1 : 2;
        return rank(a) - rank(b) || toSec(b.ts) - toSec(a.ts);
      });
      const activeCount = entries.filter(e => e.severity !== "CLEARED").length;
      return (
        <Card className="px-3.5 py-3">
          <div className="flex items-center">
            <span className="font-extrabold">Alerts</span>
            {activeCount > 0 && <span className="ml-2 text-[11px] font-bold text-white rounded-full px-[7px] py-px" style={{ background: C.red }}>{activeCount}</span>}
            <button onClick={onClear} className="ml-auto text-xs font-semibold" style={{ color: C.ind }}>Clear feed</button>
          </div>
          <div className="mt-2 flex flex-col gap-1.5 max-h-[220px] overflow-y-auto">
            {entries.length === 0 && <div className="text-[13px] py-2" style={{ color: C.dim }}>No anomalies detected</div>}
            {entries.map(e => e.severity === "CLEARED" ? (
              <div key={e.type} className="flex gap-2.5 items-center px-2.5 py-1 text-[13px]" style={{ color: C.dim }}>
                <span style={{ color: C.grn }}>✓</span>{typeLabel(e.type)} resolved
                <span className="ml-auto text-[11px] whitespace-nowrap">{fmtTime(e.ts)}</span>
              </div>
            ) : (
              <div key={e.type} className="flex gap-2.5 items-center px-2.5 py-2 rounded-[10px]"
                   style={{ background: e.severity === "CRIT" ? "#fef2f2" : "#fffbeb" }}>
                <span className="w-1 self-stretch rounded-sm" style={{ background: e.severity === "CRIT" ? C.red : C.amb }} />
                <div className="min-w-0">
                  <div className="font-bold text-[13px]">{typeLabel(e.type)} <span className="text-[11px] font-bold" style={{ color: e.severity === "CRIT" ? C.red : C.amb }}>{e.severity}</span></div>
                  <div className="text-xs" style={{ color: C.dim }}>{anomalyDetail(e)}</div>
                </div>
                <span className="ml-auto text-[11px] whitespace-nowrap" style={{ color: C.dim }}>{fmtTime(e.ts)}</span>
              </div>
            ))}
          </div>
        </Card>
      );
    }

    // ── Time charts ──
    const AXIS = { stroke: "transparent", tick: { fill: C.grey, fontSize: 10, fontFamily: "Manrope" }, tickLine: false };
    const TOOLTIP = {
      contentStyle: { background: "#fff", border: `1px solid ${C.line}`, borderRadius: 10, fontSize: 12, boxShadow: "0 4px 16px rgba(16,24,40,.08)" },
      labelFormatter: (t) => fmtTime(t),
      formatter: (v, name) => [typeof v === "number" ? Math.round(v * 10) / 10 : v, name],
    };

    function TimeChart({ id, data, series, domain, ticks, height, now, refLines = [], extendToNow = false }) {
      const from = now - CHART_WINDOW_S;
      let shown = data.filter(p => p.t >= from - 5);
      // step series only publish on change: carry the last value to "now"
      if (extendToNow && data.length) shown = [...shown.length ? shown : [{ ...data[data.length - 1], t: from }], { ...data[data.length - 1], t: now }];
      return (
        <ResponsiveContainer width="100%" height={height}>
          <AreaChart data={shown} margin={{ top: 8, right: 4, bottom: 0, left: -26 }}>
            <defs>
              {series.map(s => (
                <linearGradient key={s.key} id={`g_${id}_${s.key}`} x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor={s.color} stopOpacity={0.35} />
                  <stop offset="100%" stopColor={s.color} stopOpacity={0} />
                </linearGradient>
              ))}
            </defs>
            <CartesianGrid stroke="#f1f3f7" vertical={false} />
            <XAxis dataKey="t" type="number" domain={[from, now]} allowDataOverflow tickFormatter={fmtTime} tickCount={3} {...AXIS} />
            <YAxis domain={domain} ticks={ticks} allowDataOverflow {...AXIS} />
            <Tooltip {...TOOLTIP} />
            {refLines.map(r => <ReferenceLine key={r.y} y={r.y} stroke={r.color} strokeDasharray="4 4" />)}
            {series.map(s => (
              <Area key={s.key} type={s.type || "monotone"} dataKey={s.key} name={s.name} stroke={s.color} strokeWidth={2.5}
                    fill={`url(#g_${id}_${s.key})`} dot={false} isAnimationActive={false} />
            ))}
          </AreaChart>
        </ResponsiveContainer>
      );
    }

    function ChartCard({ title, right, children }) {
      return (
        <Card className="px-3.5 pt-3 pb-1.5">
          <div className="flex justify-between items-baseline gap-2"><span className="font-extrabold">{title}</span>{right}</div>
          {children}
        </Card>
      );
    }

    // ── Toggle switch ──
    function ToggleSwitch({ active, onToggle, label }) {
      return (
        <button type="button" role="switch" aria-checked={active} aria-label={label}
                className={`tg ${active ? "on" : ""}`} onClick={onToggle} />
      );
    }

    // ── Fault injection (simulator only) ──
    function FaultInjectionCard({ toggles, onToggle, onImuDriftReset }) {
      const [simSpeed, setSimSpeed] = useState(1.0);
      const [motorStallSeverity, setMotorStallSeverity] = useState(0.5);
      const [slamQuality, setSlamQuality] = useState(50);

      useEffect(() => {
        fetch(`${API_BASE}/live/sim_speed`).then(r => r.json())
          .then(d => { if (typeof d.sim_speed === "number") setSimSpeed(d.sim_speed); })
          .catch(() => {});
      }, []);

      const handleSimSpeedChange = (e) => {
        const val = parseFloat(e.target.value);
        setSimSpeed(val);
        post("/control/sim_speed", { speed: val }).catch(() => {});
      };

      const publishStallSeverity = (val) => post("/api/publish", {
        topic: "car/mock/motor_stall_params",
        payload: { severity: parseFloat(val.toFixed(2)) },
      }).catch(() => {});

      const slamParams = (quality) => ({
        decay: parseFloat((0.980 + (quality / 100) * (1.000 - 0.980)).toFixed(4)),
        recover: parseFloat((1.005 + (quality / 100) * (1.050 - 1.005)).toFixed(4)),
      });
      const publishSlamParams = (quality) => post("/api/publish", {
        topic: "car/mock/slam_params",
        payload: slamParams(quality),
      }).catch(() => {});

      const handleMotorStallSeverityChange = (e) => {
        const val = parseFloat(e.target.value);
        setMotorStallSeverity(val);
        if (toggles.motor_stall) publishStallSeverity(val);
      };

      const handleSlamQualityChange = (e) => {
        const val = parseInt(e.target.value, 10);
        setSlamQuality(val);
        if (toggles.slam_low_feature) publishSlamParams(val);
      };

      // Publish motor_stall severity on first activation
      const prevMotorRef = useRef(toggles.motor_stall);
      useEffect(() => {
        if (toggles.motor_stall && !prevMotorRef.current) publishStallSeverity(motorStallSeverity);
        prevMotorRef.current = toggles.motor_stall;
      }, [toggles.motor_stall, motorStallSeverity]);

      // Publish slam params on first activation
      const prevSlamRef = useRef(toggles.slam_low_feature);
      useEffect(() => {
        if (toggles.slam_low_feature && !prevSlamRef.current) publishSlamParams(slamQuality);
        prevSlamRef.current = toggles.slam_low_feature;
      }, [toggles.slam_low_feature, slamQuality]);

      const slider = (label, value, display, props, onChange, color, ends) => (
        <div className="mt-1.5">
          <div className="flex items-center gap-2.5 text-xs" style={{ color: C.dim }}>
            {label}
            <input type="range" value={value} onChange={onChange} className="flex-1" style={{ accentColor: color }} aria-label={label} {...props} />
            <b className="mono" style={{ color: C.text }}>{display}</b>
          </div>
          {ends && <div className="flex justify-between text-[10px] pl-[60px] pr-9" style={{ color: C.grey }}><span>{ends[0]}</span><span>{ends[1]}</span></div>}
        </div>
      );

      return (
        <Card className="px-3.5 py-3">
          <div className="flex justify-between items-baseline"><span className="font-extrabold">Fault injection</span><span className="k">Simulator only</span></div>
          <div className="mt-1.5">
            {ANOMALY_TYPES.map(({ key, label }) => (
              <div key={key} className="py-[7px]" style={{ borderBottom: "1px solid #f1f3f7" }}>
                <div className="flex items-center text-sm font-semibold gap-2.5">
                  <span>{label}</span>
                  {key === "imu_static_drift" && (
                    <button onClick={onImuDriftReset} className="ml-auto text-xs font-semibold" style={{ color: C.ind }}>Reset IMU</button>
                  )}
                  <span className={key === "imu_static_drift" ? "" : "ml-auto"}>
                    <ToggleSwitch active={toggles[key]} onToggle={() => onToggle(key)} label={label} />
                  </span>
                </div>
                {key === "motor_stall" && toggles[key] && slider("Severity", motorStallSeverity, motorStallSeverity.toFixed(2),
                  { min: 0, max: 1, step: 0.01 }, handleMotorStallSeverityChange, C.red, ["0.0 arc", "1.0 freeze"])}
                {key === "slam_low_feature" && toggles[key] && slider("Room quality", slamQuality, `${slamQuality}%`,
                  { min: 0, max: 100 }, handleSlamQualityChange, C.amb, ["0%", "100%"])}
              </div>
            ))}
          </div>
          <div className="flex items-center gap-2.5 pt-2.5 text-sm font-semibold">
            Sim speed
            <input type="range" min="0.25" max="4.0" step="0.25" value={simSpeed} onChange={handleSimSpeedChange}
                   className="flex-1" style={{ accentColor: C.ind }} aria-label="Simulation speed" />
            <span className="mono text-[13px]">{simSpeed.toFixed(2)}×</span>
          </div>
        </Card>
      );
    }

    // ─────────────────────────────────────────────
    // MAIN APP
    // ─────────────────────────────────────────────
    function App() {
      const { anomalies, add: addAnomaly, clear: clearAnomalies } = useAnomalies();
      const { connected, snapshot } = useWebSocket(addAnomaly);

      const liveSlam   = snapshot["car/slam/pose"] || null;
      const liveImu    = snapshot["car/imu"] || null;
      const liveMotors = snapshot["car/motors"] || null;
      const livePhase  = snapshot["car/nav/phase"] || null;

      const [slamHistory]   = useHistory("/history/slam?minutes=10");
      const [imuHistory]    = useHistory("/history/imu?minutes=10");
      const [motorHistory]  = useHistory("/history/motors?minutes=10");
      const [phaseHistory]  = useHistory("/history/phase?minutes=10");

      const slamRate = usePoll("/live/slam_rate", 500, { msgs_per_sec: 0, status: "critical" });

      // car clock (sim/Pi timestamps) so charts line up with the data even if the browser clock differs
      const carNow = liveImu?.ts ?? liveSlam?.ts ?? Date.now() / 1000;
      const browserNow = Date.now() / 1000;

      const { path, lastGood } = usePath(slamHistory, liveSlam);
      const { segments, leg, resetDocked } = useNav(phaseHistory, livePhase);

      const headingSeries = useSeries(imuHistory, liveImu,
        r => r.heading_deg == null ? null : { t: toSec(r.ts), heading: norm360(r.heading_deg) }, 0.25);
      const motorSeries = useSeries(motorHistory, liveMotors,
        r => r.total == null ? null : { t: toSec(r.ts), total: r.total, inner: r.inner }, 0);

      const [rateSeries, setRateSeries] = useState([]);
      useEffect(() => {
        setRateSeries(prev => [...prev, { t: Date.now() / 1000, rate: slamRate.msgs_per_sec }].slice(-MAX_CHART_POINTS));
      }, [slamRate]);

      const [toggles, setToggles] = useState(
        Object.fromEntries(ANOMALY_TYPES.map(a => [a.key, false]))
      );

      const handleToggle = useCallback(async (key) => {
        const newVal = !toggles[key];
        setToggles(prev => ({ ...prev, [key]: newVal }));
        try {
          await post("/api/inject", { anomaly: key, active: newVal });
        } catch (e) {
          console.error("Inject failed:", e);
          setToggles(prev => ({ ...prev, [key]: !newVal }));
        }
      }, [toggles]);

      const handleImuDriftReset = useCallback(() => {
        post("/control/imu_drift_reset").catch(() => {});
      }, []);

      const handleGoto = useCallback((name) => {
        post("/api/goto", { target: name }).catch(() => {});
      }, []);

      const handleReset = useCallback(() => {
        post("/sim/reset").catch(() => {});
        resetDocked();
      }, [resetDocked]);

      const stallActive = anomalies.motor_stall && anomalies.motor_stall.severity !== "CLEARED";
      const currentSeg = segments[segments.length - 1];
      const phaseNow = livePhase?.phase ?? currentSeg?.phase;

      const mapProps = {
        path, pose: liveSlam, lastGood, heading: liveImu?.heading_deg ?? null,
        leg, phase: phaseNow, alert: !!stallActive, now: carNow,
      };

      return (
        <div className="min-h-screen lg:h-screen flex flex-col gap-3 p-3 sm:px-[18px] sm:py-3.5">
          <Header wsConnected={connected} slamSeq={liveSlam?.seq ?? null} />
          <AlertBanner anomalies={anomalies} onClear={clearAnomalies} />
          <KpiRow slam={liveSlam} slamRate={slamRate} phase={livePhase} phaseSince={currentSeg?.start}
                  motors={liveMotors} imu={liveImu} now={carNow} />

          <div className="flex-1 min-h-0 grid grid-cols-1 lg:grid-cols-[minmax(0,1fr)_380px] gap-3">
            <MapCard mapProps={mapProps} currentTarget={NAV_PHASES.has(phaseNow) ? livePhase?.target_station : null}
                     onGoto={handleGoto} onReset={handleReset} segments={segments} now={carNow} />

            <aside className="flex flex-col gap-3 lg:min-h-0 lg:overflow-y-auto overflow-x-hidden lg:pr-1">
              <AlertsCard anomalies={anomalies} onClear={clearAnomalies} />
              <ChartCard title="Heading"
                         right={<span className="font-extrabold" style={{ color: C.vio }}>{liveImu?.heading_deg != null ? `${Math.round(norm360(liveImu.heading_deg))}°` : "—"}</span>}>
                <TimeChart id="hdg" data={headingSeries} now={carNow} height={84} domain={[0, 360]} ticks={[0, 180, 360]}
                           series={[{ key: "heading", name: "Heading °", color: C.vio }]} />
              </ChartCard>
              <ChartCard title="Motor PWM"
                         right={<span className="k"><span style={{ color: C.ind }}>●</span> total <span style={{ color: C.indSoft }}>●</span> inner</span>}>
                <TimeChart id="mot" data={motorSeries} now={carNow} height={84} domain={[0, 60]} ticks={[0, 30, 60]} extendToNow
                           series={[{ key: "total", name: "Total", color: C.ind, type: "stepAfter" },
                                    { key: "inner", name: "Inner", color: C.indSoft, type: "stepAfter" }]} />
              </ChartCard>
              <ChartCard title="SLAM publish rate"
                         right={<span className="k">warn &lt; 20 · crit &lt; 10</span>}>
                <TimeChart id="rate" data={rateSeries} now={browserNow} height={74} domain={[0, 35]} ticks={[0, 10, 20, 30]}
                           refLines={[{ y: 20, color: C.amb }, { y: 10, color: C.red }]}
                           series={[{ key: "rate", name: "msg/s", color: { ok: C.grn, warn: C.amb, critical: C.red }[slamRate.status] || C.grn }]} />
              </ChartCard>
              <FaultInjectionCard toggles={toggles} onToggle={handleToggle} onImuDriftReset={handleImuDriftReset} />
            </aside>
          </div>
        </div>
      );
    }

    // ─────────────────────────────────────────────
    // MOUNT
    // ─────────────────────────────────────────────
    const root = ReactDOM.createRoot(document.getElementById("root"));
    root.render(<App />);
