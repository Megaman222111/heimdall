// Dashboard page served at "/" by grove_follow.ino. Polls /state for the latest
// camera frame + steering math, and posts tuning changes to /set.
#pragma once

static const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Person Follower</title>
<style>
:root{--page:#f9f9f7;--surface:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--grid:#e4e3df;--band:#f0efec;
--series:#2a78d6;--good:#0ca30c;--critical:#d03b3b;--warn:#fab219}
@media (prefers-color-scheme:dark){:root{--page:#0d0d0d;--surface:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;
--grid:#2e2e2c;--band:#383835;--series:#3987e5}}
*{box-sizing:border-box}
body{margin:0;background:var(--page);color:var(--ink);font:15px/1.4 system-ui,-apple-system,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:12px 16px}
h1{font-size:18px;margin:0;flex:1}
.pill{font-size:13px;color:var(--ink2);display:flex;align-items:center;gap:6px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--critical)}
.dot.on{background:var(--good)}
main{display:grid;gap:16px;padding:0 16px 24px;grid-template-columns:minmax(0,3fr) minmax(0,2fr)}
@media (max-width:820px){main{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--surface);border:1px solid var(--grid);border-radius:10px;padding:14px}
.card h2{font-size:13px;font-weight:600;color:var(--ink2);margin:0 0 10px;text-transform:uppercase;letter-spacing:.04em}
#cam{width:100%;height:auto;display:block;border-radius:6px;background:#000;image-rendering:auto}
.camnote{font-size:13px;color:var(--ink2);margin-top:8px}
.math{font:13px/1.7 ui-monospace,Menlo,monospace;white-space:pre-wrap;margin:0}
.math b{font-weight:600}
.state{display:inline-flex;align-items:center;gap:6px;font-weight:600}
.state i{width:10px;height:10px;border-radius:50%;display:inline-block}
.dial{display:flex;align-items:center;gap:16px}
.dial svg{width:150px;flex:none}
.big{font-size:34px;font-weight:600;font-variant-numeric:tabular-nums}
.sub{font-size:13px;color:var(--ink2)}
.charts{grid-column:1/-1;display:grid;gap:16px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.chart{position:relative}
.chart canvas{width:100%;height:170px;display:block}
.tip{position:absolute;pointer-events:none;background:var(--ink);color:var(--surface);font-size:12px;
padding:4px 8px;border-radius:5px;white-space:nowrap;display:none;transform:translate(-50%,-120%)}
form{display:grid;grid-template-columns:auto 1fr auto;gap:8px 10px;align-items:center;font-size:14px}
form input[type=range]{width:100%}
form output{font-variant-numeric:tabular-nums;min-width:3.5em;text-align:right}
.saved{font-size:12px;color:var(--ink2);grid-column:1/-1}
</style></head><body>
<header><h1>Person Follower</h1>
<span class="pill"><span class="dot" id="link"></span><span id="linktxt">connecting…</span></span></header>
<main>
  <section class="card"><h2>What the Grove sees</h2>
    <canvas id="cam" width="240" height="240"></canvas>
    <div class="camnote" id="camnote">Waiting for the first frame…</div>
  </section>
  <div style="display:grid;gap:16px;align-content:start">
    <section class="card"><h2>Servo</h2>
      <div class="dial">
        <svg viewBox="0 0 160 92" aria-hidden="true">
          <path d="M10 82 A70 70 0 0 1 150 82" fill="none" stroke="var(--grid)" stroke-width="10" stroke-linecap="round"/>
          <line id="needle" x1="80" y1="82" x2="80" y2="20" stroke="var(--series)" stroke-width="4" stroke-linecap="round"/>
          <circle cx="80" cy="82" r="6" fill="var(--ink)"/>
          <text x="10" y="92" font-size="9" fill="var(--ink2)">0°</text>
          <text x="139" y="92" font-size="9" fill="var(--ink2)">180°</text>
        </svg>
        <div><div class="big" id="angle">–</div><div class="sub" id="state">–</div></div>
      </div>
    </section>
    <section class="card"><h2>Steering math (this frame)</h2><pre class="math" id="math">–</pre></section>
    <section class="card"><h2>Tuning</h2>
      <form id="tune">
        <label for="gain">Gain</label><input type="range" id="gain" min="0.5" max="15" step="0.5"><output id="gainv"></output>
        <label for="dead">Dead zone</label><input type="range" id="dead" min="0" max="0.3" step="0.01"><output id="deadv"></output>
        <label for="max">Max step</label><input type="range" id="max" min="0.5" max="15" step="0.5"><output id="maxv"></output>
        <label for="invert">Invert</label><input type="checkbox" id="invert" style="justify-self:start"><span></span>
        <span class="saved" id="saved">Changes apply instantly and are saved on the ESP32.</span>
      </form>
    </section>
  </div>
  <div class="charts">
    <section class="card"><h2>Error (where you are, −1 left … +1 right) — last 30 s</h2>
      <div class="chart"><canvas id="cErr"></canvas><div class="tip" id="tErr"></div></div></section>
    <section class="card"><h2>Servo angle (°) — last 30 s</h2>
      <div class="chart"><canvas id="cAng"></canvas><div class="tip" id="tAng"></div></div></section>
  </div>
</main>
<script>
const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const HIST_MS = 30000;
const hist = [];
const img = new Image();
let lastId = -1, cfgLoaded = false, last = null;

async function poll() {
  try {
    const s = await (await fetch('/state', {cache: 'no-store'})).json();
    link(true, s.fps ? s.fps.toFixed(1) + ' fps' : 'connected');
    if (s.id !== lastId) { lastId = s.id; onFrame(s); }
  } catch (e) { link(false, 'offline — retrying'); }
  setTimeout(poll, 80);
}
function link(on, txt) { $('link').classList.toggle('on', on); $('linktxt').textContent = txt; }

function onFrame(s) {
  last = s;
  if (s.image) { img.onload = () => drawCam(s); img.src = 'data:image/jpeg;base64,' + s.image; }
  else drawCam(s);
  showMath(s);
  if (!cfgLoaded) loadCfg(s.cfg);
  hist.push({t: s.ms, err: s.found ? s.error : null, ang: s.angle});
  while (hist.length && s.ms - hist[0].t > HIST_MS) hist.shift();
  drawCharts();
}

function drawCam(s) {
  const c = $('cam'), g = c.getContext('2d');
  const W = s.w, H = s.h;
  if (c.width !== W || c.height !== H) { c.width = W; c.height = H; }
  if (img.complete && img.naturalWidth) g.drawImage(img, 0, 0, W, H);
  else { g.fillStyle = '#000'; g.fillRect(0, 0, W, H); }
  const cx = W / 2, band = s.cfg.dead * W / 2, lw = Math.max(1, W / 240);
  const line = (x1, y1, x2, y2, color, w) => {
    g.strokeStyle = 'rgba(0,0,0,.6)'; g.lineWidth = w + 2 * lw; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
    g.strokeStyle = color; g.lineWidth = w; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
  };
  line(cx, 0, cx, H, '#fff', lw);
  line(cx - band, 0, cx - band, H, css('--warn'), lw);
  line(cx + band, 0, cx + band, H, css('--warn'), lw);
  const ok = s.found && Math.abs(s.error) < s.cfg.dead;
  const tColor = ok ? css('--good') : css('--critical');
  for (const p of s.people) {
    const [x, y, w, h, sc] = p.b;
    g.strokeStyle = p.t ? tColor : 'rgba(255,255,255,.7)'; g.lineWidth = (p.t ? 2 : 1) * lw;
    g.strokeRect(x - w / 2, y - h / 2, w, h);
    const label = `${p.t ? 'target ' : ''}${sc}%`, lx = x - w / 2, ly = y - h / 2;
    g.font = `${11 * lw}px system-ui`;
    g.fillStyle = 'rgba(0,0,0,.65)'; g.fillRect(lx, ly, g.measureText(label).width + 6 * lw, 15 * lw);
    g.fillStyle = p.t ? tColor : '#fff'; g.fillText(label, lx + 3 * lw, ly + 11 * lw);
    g.fillStyle = g.strokeStyle;
    if (p.k) for (const [kx, ky, ks] of p.k) if (ks >= 50) { g.beginPath(); g.arc(kx, ky, 2.5 * lw, 0, 7); g.fill(); }
  }
  if (s.found) {
    const ty = s.people.find(p => p.t)?.b[1] ?? H / 2;
    line(cx, ty, s.x, ty, tColor, 2 * lw);
    g.fillStyle = tColor; g.beginPath(); g.arc(s.x, ty, 5 * lw, 0, 7); g.fill();
  }
  $('camnote').textContent = s.image || img.naturalWidth
    ? `${W}×${H} px · ${s.people.length} detected · white = center, yellow = dead zone`
    : 'No image yet (the camera feed starts once this page is open).';
}

function f(v, d = 2) { return (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(d); }
function showMath(s) {
  const c = s.cfg, half = s.w / 2;
  $('angle').textContent = s.sent + '°';
  $('needle').setAttribute('transform', `rotate(${s.angle - 90} 80 82)`);
  const st = !s.found ? ['No person — holding', css('--ink2')]
    : Math.abs(s.error) < c.dead ? ['Centered', css('--good')] : ['Turning toward target', css('--critical')];
  $('state').innerHTML = `<span class="state"><i style="background:${st[1]}"></i>${st[0]}</span>`;
  if (!s.found) { $('math').textContent = `No person in view.\nHolding at ${s.sent}°.`; return; }
  const raw = c.gain * s.error, delta = c.invert ? -s.step : s.step;
  $('math').innerHTML =
    `x      = <b>${s.x.toFixed(0)}</b> px   (frame width ${s.w})\n` +
    `error  = (x − W/2) / (W/2)\n       = (${s.x.toFixed(0)} − ${half}) / ${half} = <b>${f(s.error)}</b>\n` +
    (Math.abs(s.error) < c.dead
      ? `|error| < dead zone ${c.dead.toFixed(2)} → <b>step = 0</b>\n`
      : `step   = clamp(gain × error, ±max)\n       = clamp(${c.gain.toFixed(1)} × ${f(s.error)}, ±${c.max.toFixed(1)})\n` +
        `       = clamp(${f(raw)}) = <b>${f(s.step)}°</b>\n`) +
    (c.invert ? `invert → <b>${f(delta)}°</b>\n` : '') +
    `angle  = ${s.before.toFixed(1)} ${delta < 0 ? '−' : '+'} ${Math.abs(delta).toFixed(2)} = ${s.angle.toFixed(1)}°\n` +
    `sent to Nano → <b>"${s.sent}"</b>`;
}

function drawCharts() {
  chart($('cErr'), $('tErr'), 'err', -1, 1, {band: last ? last.cfg.dead : 0, ticks: [-1, 0, 1], fmt: v => f(v)});
  chart($('cAng'), $('tAng'), 'ang', 0, 180, {ticks: [0, 90, 180], fmt: v => v.toFixed(1) + '°'});
}
const hover = {};
function chart(c, tip, key, lo, hi, o) {
  const dpr = devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
  if (c.width !== W * dpr) { c.width = W * dpr; c.height = H * dpr; }
  const g = c.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const L = 40, R = 8, T = 8, B = 20, pw = W - L - R, ph = H - T - B;
  const now = hist.length ? hist[hist.length - 1].t : 0;
  const X = t => L + pw * (1 - (now - t) / HIST_MS), Y = v => T + ph * (1 - (v - lo) / (hi - lo));
  g.font = '11px system-ui'; g.fillStyle = css('--ink2'); g.textAlign = 'right'; g.textBaseline = 'middle';
  if (o.band) { g.fillStyle = css('--band'); g.fillRect(L, Y(o.band), pw, Y(-o.band) - Y(o.band)); g.fillStyle = css('--ink2'); }
  for (const v of o.ticks) {
    g.strokeStyle = css('--grid'); g.lineWidth = 1; g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke();
    g.fillText(key === 'err' ? (v > 0 ? '+' : '') + v : v + '°', L - 6, Y(v));
  }
  g.textAlign = 'center'; g.textBaseline = 'alphabetic';
  g.fillText('−30 s', L + 14, H - 4); g.fillText('now', W - R - 10, H - 4);
  g.strokeStyle = css('--series'); g.lineWidth = 2; g.lineJoin = g.lineCap = 'round'; g.beginPath();
  let pen = false;
  for (const p of hist) {
    if (p[key] == null) { pen = false; continue; }
    const x = X(p.t), y = Y(Math.max(lo, Math.min(hi, p[key])));
    pen ? g.lineTo(x, y) : g.moveTo(x, y); pen = true;
  }
  g.stroke();
  const hx = hover[c.id];
  if (hx == null || !hist.length) { tip.style.display = 'none'; return; }
  let best = null;
  for (const p of hist) if (p[key] != null && (!best || Math.abs(X(p.t) - hx) < Math.abs(X(best.t) - hx))) best = p;
  if (!best) { tip.style.display = 'none'; return; }
  const bx = X(best.t), by = Y(best[key]);
  g.strokeStyle = css('--ink2'); g.lineWidth = 1; g.beginPath(); g.moveTo(bx, T); g.lineTo(bx, T + ph); g.stroke();
  g.fillStyle = css('--series'); g.strokeStyle = css('--surface'); g.lineWidth = 2;
  g.beginPath(); g.arc(bx, by, 4, 0, 7); g.fill(); g.stroke();
  tip.style.display = 'block'; tip.style.left = bx + 'px'; tip.style.top = by + 'px';
  tip.textContent = `${o.fmt(best[key])} · ${((now - best.t) / 1000).toFixed(1)} s ago`;
}
for (const id of ['cErr', 'cAng']) {
  const c = $(id);
  c.addEventListener('pointermove', e => { hover[id] = e.offsetX; drawCharts(); });
  c.addEventListener('pointerleave', () => { delete hover[id]; drawCharts(); });
}
addEventListener('resize', drawCharts);

function loadCfg(c) {
  $('gain').value = c.gain; $('dead').value = c.dead; $('max').value = c.max; $('invert').checked = c.invert;
  outs(); cfgLoaded = true;
}
function outs() {
  $('gainv').textContent = (+$('gain').value).toFixed(1);
  $('deadv').textContent = (+$('dead').value).toFixed(2);
  $('maxv').textContent = (+$('max').value).toFixed(1) + '°';
}
let saveTimer;
$('tune').addEventListener('input', () => {
  outs(); clearTimeout(saveTimer);
  saveTimer = setTimeout(async () => {
    const q = new URLSearchParams({gain: $('gain').value, dead: $('dead').value, max: $('max').value, invert: $('invert').checked ? 1 : 0});
    try { await fetch('/set?' + q); $('saved').textContent = 'Saved ✓'; }
    catch (e) { $('saved').textContent = 'Could not save — check the connection.'; }
  }, 250);
});
poll();
</script></body></html>)HTML";
