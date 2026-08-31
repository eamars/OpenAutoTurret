"""Operator dashboard (architecture §42.1, §42.2, §42.3).

A single self-contained HTML page (inline CSS + JS, no build step) with the
§42.1 status panels and the §42.2 developer controls. It connects to the
``/ws`` WebSocket for live telemetry and POSTs to ``/api/command`` for
developer commands.

SAFETY: the page only DISPLAYS state and SUBMITS high-level commands. Every
command is validated by controld before it runs; the page has no motor/CAN
authority of its own. The video preview (§42.3) is a placeholder: video frames
are a separate low-priority path and do not traverse the control IPC.
"""
from __future__ import annotations

DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__ — OpenAutoTurret</title>
<style>
  :root { --bg:#0e1116; --panel:#161b22; --border:#2b3240; --fg:#e6edf3;
          --dim:#8b949e; --ok:#3fb950; --warn:#d29922; --err:#f85149; --accent:#58a6ff; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
  header { display:flex; align-items:center; gap:12px; padding:10px 16px;
           border-bottom:1px solid var(--border); background:var(--panel); }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  .dot { width:10px; height:10px; border-radius:50%; background:var(--err); }
  .dot.on { background:var(--ok); }
  #grid { display:grid; gap:12px; padding:12px;
          grid-template-columns:repeat(auto-fit, minmax(280px, 1fr)); }
  .panel { background:var(--panel); border:1px solid var(--border);
           border-radius:8px; padding:12px; }
  .panel h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em;
              color:var(--dim); margin:0 0 8px; }
  .row { display:flex; justify-content:space-between; padding:2px 0;
         font-variant-numeric:tabular-nums; }
  .row .k { color:var(--dim); }
  .badge { display:inline-block; padding:1px 8px; border-radius:10px;
           font-size:12px; font-weight:600; background:#222b38; }
  .badge.ok { background:rgba(63,185,80,.18); color:var(--ok); }
  .badge.warn { background:rgba(210,153,34,.18); color:var(--warn); }
  .badge.err { background:rgba(248,81,73,.18); color:var(--err); }
  .badge.info { background:rgba(88,166,255,.18); color:var(--accent); }
  #controls { grid-column:1/-1; display:flex; flex-wrap:wrap; gap:8px; }
  button { background:#21262d; color:var(--fg); border:1px solid var(--border);
           border-radius:6px; padding:7px 12px; cursor:pointer; font-size:13px; }
  button:hover { border-color:var(--accent); }
  button.primary { background:var(--accent); color:#04121f; border-color:var(--accent); }
  button.danger { background:transparent; color:var(--err); border-color:var(--err); }
  button:disabled { opacity:.45; cursor:not-allowed; }
  input { background:#0d1117; border:1px solid var(--border); color:var(--fg);
          border-radius:6px; padding:6px 8px; width:90px; }
  #log { grid-column:1/-1; font-family:ui-monospace, monospace; font-size:12px;
         max-height:160px; overflow:auto; white-space:pre-wrap; }
  .muted { color:var(--dim); }
  .video-ph { display:flex; align-items:center; justify-content:center; height:140px;
              border:1px dashed var(--border); border-radius:6px; color:var(--dim); }
</style>
</head>
<body>
<header>
  <span class="dot" id="conn"></span>
  <h1>__TITLE__ · OpenAutoTurret</h1>
  <span class="muted" id="state-word">connecting…</span>
</header>
<div id="grid">

  <section class="panel" id="p-system">
    <h2>System</h2>
    <div class="row"><span class="k">Track state</span><span class="badge" id="track-state">—</span></div>
    <div class="row"><span class="k">Safety action</span><span class="badge" id="safety">—</span></div>
    <div class="row"><span class="k">Tracking active</span><span id="tracking-active">—</span></div>
    <div class="row"><span class="k">Feedback age</span><span id="fb-age">—</span></div>
    <div class="row"><span class="k">Control cycle</span><span id="cycle">—</span></div>
  </section>

  <section class="panel" id="p-vision">
    <h2>Vision</h2>
    <div class="row"><span class="k">Target confidence</span><span id="conf">—</span></div>
    <div class="row"><span class="k">Target az (world)</span><span id="taz">—</span></div>
    <div class="row"><span class="k">Target el (world)</span><span id="tel">—</span></div>
  </section>

  <section class="panel" id="p-yawpitch">
    <h2>Yaw · Pitch</h2>
    <div class="row"><span class="k">q yaw</span><span id="qy">—</span></div>
    <div class="row"><span class="k">q ref yaw</span><span id="qry">—</span></div>
    <div class="row"><span class="k">v yaw</span><span id="vy">—</span></div>
    <div class="row"><span class="k">q pitch</span><span id="qp">—</span></div>
    <div class="row"><span class="k">q ref pitch</span><span id="qrp">—</span></div>
    <div class="row"><span class="k">v pitch</span><span id="vp">—</span></div>
  </section>

  <section class="panel" id="p-cal">
    <h2>Calibration</h2>
    <div class="row"><span class="k">Installation</span><span class="badge" id="calib">—</span></div>
    <div class="row"><span class="k">Source</span><span id="calib-src">—</span></div>
    <div class="row"><span class="k">Base roll</span><span id="br">—</span></div>
    <div class="row"><span class="k">Base pitch</span><span id="bp">—</span></div>
    <div class="row"><span class="k">Base yaw</span><span id="by">—</span></div>
  </section>

  <section class="panel" id="p-video">
    <h2>Video (low priority, §42.3)</h2>
    <div class="video-ph">video preview is a separate path — not part of the control IPC</div>
  </section>

  <section class="panel" id="p-can">
    <h2>CAN / Actuation</h2>
    <div class="row"><span class="k">Effort yaw (Nm)</span><span id="ey">—</span></div>
    <div class="row"><span class="k">Effort pitch (Nm)</span><span id="ep">—</span></div>
    <div class="muted">read-only; motors are driven solely by controld</div>
  </section>

  <section class="panel" id="p-controls" style="grid-column:1/-1">
    <h2>Developer controls (§42.2) — every request is validated by controld</h2>
    <div id="controls">
      <button class="primary" data-cmd="hold">Hold</button>
      <button data-cmd="start_tracking">Start tracking</button>
      <button data-cmd="stop_tracking">Stop tracking</button>
      <button data-cmd="enable_search">Enable search</button>
      <button data-cmd="disable_search">Disable search</button>
      <span>target
        <input id="sel-target" type="number" min="0" max="15" value="0" style="width:56px">
      </span>
      <button data-cmd="select_target">Select target</button>
      <button data-cmd="start_homing">Start homing</button>
      <button data-cmd="start_installation_calibration">Start visual calibration</button>
      <button data-cmd="start_payload_verification">Start payload verification</button>
      <span>test motion (rad)
        <input id="test-motion" type="number" step="0.05" value="0.3" style="width:80px">
      </span>
      <button data-cmd="run_test_motion">Run test motion</button>
      <button class="danger" data-cmd="request_park">Safe park</button>
      <button class="danger" data-cmd="request_shutdown">Safe shutdown</button>
    </div>
  </section>

  <section class="panel" style="grid-column:1/-1">
    <h2>Command log</h2>
    <div id="log"></div>
  </section>
</div>

<script>
const $ = (id) => document.getElementById(id);
const rad = (x) => (isFinite(x) ? (x * 57.29577951308232).toFixed(2) + "°" : "—");
const num = (x, d=4) => (isFinite(x) ? Number(x).toFixed(d) : "—");

function badge(el, text, kind) {
  el.textContent = text;
  el.className = "badge" + (kind ? " " + kind : "");
}
function safetyKind(s) {
  return s === "ALLOW" ? "ok" : s === "DERATE" ? "warn" : s === "HOLD" ? "info"
       : (s === "BRAKE" ? "warn" : "err");
}
function trackKind(s) {
  return (s === "tracking" || s === "coasting") ? "ok"
       : s === "search" ? "info" : s === "target_lost" ? "warn" : "info";
}

let ws = null;
function connect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(proto + location.host + "/ws");
  ws.onopen = () => { $("conn").classList.add("on"); $("state-word").textContent = "connected"; };
  ws.onclose = () => { $("conn").classList.remove("on"); $("state-word").textContent = "disconnected — retrying"; setTimeout(connect, 1500); };
  ws.onmessage = (e) => {
    let t; try { t = JSON.parse(e.data); } catch { return; }
    if (t.type !== "telemetry") return;
    render(t);
  };
}
function render(t) {
  badge($("track-state"), t.track_state, trackKind(t.track_state));
  badge($("safety"), t.safety_action, safetyKind(t.safety_action));
  $("tracking-active").textContent = t.tracking_active ? "yes" : "no";
  $("fb-age").textContent = num(t.feedback_age_ms, 1) + " ms";
  $("cycle").textContent = num(t.control_cycle_us, 1) + " µs";
  $("conf").textContent = num(t.target_confidence, 3);
  $("taz").textContent = rad(t.target_az_world_rad);
  $("tel").textContent = rad(t.target_el_world_rad);
  $("qy").textContent = num(t.q_yaw_rad);
  $("qry").textContent = num(t.q_ref_yaw_rad);
  $("vy").textContent = num(t.v_yaw_rad_s, 3);
  $("qp").textContent = num(t.q_pitch_rad);
  $("qrp").textContent = num(t.q_ref_pitch_rad);
  $("vp").textContent = num(t.v_pitch_rad_s, 3);
  const calib = t.installation_calibrated ? "calibrated" : "NOT calibrated";
  badge($("calib"), calib, t.installation_calibrated ? "ok" : "warn");
  $("calib-src").textContent = t.installation_source;
  $("br").textContent = rad(t.base_roll_rad);
  $("bp").textContent = rad(t.base_pitch_rad);
  $("by").textContent = rad(t.base_yaw_rad);
  $("ey").textContent = num(t.effort_yaw, 2);
  $("ep").textContent = num(t.effort_pitch, 2);
}

function logline(msg, kind) {
  const el = $("log");
  const time = new Date().toLocaleTimeString();
  el.textContent = `[${time}] ${msg}\n` + el.textContent;
  if (el.children.length > 200) el.textContent = el.textContent.split("\n").slice(0, 200).join("\n");
}

document.getElementById("controls").addEventListener("click", async (ev) => {
  const btn = ev.target.closest("button[data-cmd]");
  if (!btn) return;
  const cmd = btn.dataset.cmd;
  let arg = "";
  if (cmd === "select_target") arg = String($("sel-target").value || 0);
  if (cmd === "run_test_motion") arg = String($("test-motion").value || 0);
  btn.disabled = true;
  try {
    const r = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: cmd, arg }),
    });
    const j = await r.json();
    if (j.ok) logline(`${cmd}(${arg}) -> OK`, "ok");
    else logline(`${cmd}(${arg}) -> REJECTED: ${j.error}`, "err");
  } catch (e) {
    logline(`${cmd} -> ERROR: ${e}`, "err");
  } finally {
    btn.disabled = false;
  }
});

connect();
</script>
</body>
</html>
"""


def dashboard_html(title: str = "OpenAutoTurret") -> str:
    """Return the dashboard HTML with the given title substituted."""
    return DASHBOARD_HTML.replace("__TITLE__", title)
