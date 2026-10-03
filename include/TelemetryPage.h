#pragma once
#include <pgmspace.h>

// Self-contained page: no external libraries, so it works without internet.
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Balancer Telemetry</title>
<style>
  body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px}
  canvas{width:100%;height:320px;background:#1b1b1b;border-radius:8px;display:block}
  .row{display:flex;gap:12px;flex-wrap:wrap;margin-top:12px;align-items:flex-end}
  label{display:flex;flex-direction:column;font-size:14px}
  input{width:90px;padding:4px}
  button{padding:6px 14px}
  .legend span{margin-right:14px;font-size:14px}
</style>
</head>
<body>
<h3>Self-Balancing Robot <small id="st">connecting...</small></h3>
<canvas id="c"></canvas>
<div id="dbg" style="font-size:12px;color:#888;margin-top:6px">msgs: 0 (waiting for data)</div>
<div class="legend" style="margin-top:8px">
  <span style="color:#4fc3f7">&#9632; angle (&deg;)</span>
  <span style="color:#aed581">&#9632; setpoint (&deg;)</span>
  <span style="color:#ffb74d">&#9632; motor output</span>
</div>
<div class="row">
  <label>Kp <input id="kp" type="number" step="0.1" onchange="sendParam('kp')"></label>
  <label>Ki <input id="ki" type="number" step="0.01" onchange="sendParam('ki')"></label>
  <label>Kd <input id="kd" type="number" step="0.01" onchange="sendParam('kd')"></label>
  <button onclick="togglePause()" id="pb">Pause</button>
</div>

<script>
const N = 500;  // samples shown (~10 s at 50 Hz)
const series = [
  {c:'#4fc3f7', d:[]},  // angle
  {c:'#aed581', d:[]},  // setpoint
  {c:'#ffb74d', d:[]}   // output
];
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
const st = document.getElementById('st');
const dbg = document.getElementById('dbg');
let ws, paused = false, msgCount = 0;

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen  = () => st.textContent = '(connected)';
  ws.onclose = () => { st.textContent = '(disconnected)'; setTimeout(connect, 1000); };
  ws.onmessage = e => {
    msgCount++;
    const lines = e.data.split('\n').filter(l => l.length);
    dbg.textContent = 'msgs: ' + msgCount + ' | last: ' + lines[lines.length - 1];
    if (paused) return;
    lines.forEach(line => {
      const v = line.split(',').map(Number);  // t, angle, setpoint, output
      if (!v.every(Number.isFinite)) return;  // skip nan/inf values
      for (let i = 0; i < 3; i++) {
        const d = series[i].d;
        d.push(v[i + 1]);
        if (d.length > N) d.shift();
      }
    });
  };
}

function sendParam(name) {
  if (ws.readyState === 1) ws.send(name + '=' + document.getElementById(name).value);
}
function togglePause() {
  paused = !paused;
  document.getElementById('pb').textContent = paused ? 'Resume' : 'Pause';
}

function plot(group, x, y, w, h) {
  let min = Infinity, max = -Infinity;
  group.forEach(i => series[i].d.forEach(v => { min = Math.min(min, v); max = Math.max(max, v); }));
  if (!isFinite(min)) return;
  if (max - min < 1e-3) { max += 1; min -= 1; }
  const pad = (max - min) * 0.1; min -= pad; max += pad;

  ctx.strokeStyle = '#333'; ctx.strokeRect(x, y, w, h);
  ctx.fillStyle = '#888';
  ctx.fillText(max.toFixed(1), x + 4, y + 12);
  ctx.fillText(min.toFixed(1), x + 4, y + h - 4);

  group.forEach(i => {
    ctx.strokeStyle = series[i].c; ctx.beginPath();
    series[i].d.forEach((v, k) => {
      const px = x + k * w / (N - 1), py = y + h - (v - min) / (max - min) * h;
      k ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
    });
    ctx.stroke();
  });
}

function draw() {
  const r = cv.getBoundingClientRect();
  cv.width = r.width; cv.height = r.height;
  plot([0, 1], 0, 0, cv.width, cv.height * 0.55 - 4);          // angle + setpoint
  plot([2], 0, cv.height * 0.55, cv.width, cv.height * 0.45);   // output
  requestAnimationFrame(draw);
}

connect();
draw();
</script>
</body>
</html>
)rawliteral";
