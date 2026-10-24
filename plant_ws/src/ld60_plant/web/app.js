'use strict';
/* LD-60 AMR Planner - talks to amr_manager through rosbridge (JSON over std_msgs/String). */

const S = { ws: null, connected: false, state: null, map: null, pending: new Map(), seq: 0,
  view: null, follow: false, speedDragging: false };
const $ = (s) => document.querySelector(s);
const NS = 'http://www.w3.org/2000/svg';
const svgEl = (tag, attrs = {}, parent = null) => {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (parent) parent.append(e);
  return e;
};
const fmt = (v, d = 2) => (v == null ? '—' : Number(v).toFixed(d));

/* ================================================================ rosbridge */
function connect() {
  const host = location.hostname || 'localhost';
  $('#wsHost').textContent = host;
  const ws = new WebSocket(`ws://${host}:9090`);
  S.ws = ws;
  ws.onopen = () => {
    S.connected = true;
    $('#offline').hidden = true;
    send({ op: 'advertise', topic: '/amr/cmd', type: 'std_msgs/msg/String' });
    send({ op: 'advertise', topic: '/plant/cmd', type: 'std_msgs/msg/String' });
    for (const t of ['/amr/state', '/amr/map', '/amr/reply', '/amr/log', '/plant/state', '/plant/reply', '/plant/log']) {
      send({ op: 'subscribe', topic: t, type: 'std_msgs/msg/String', queue_length: 50 });
    }
    setTimeout(() => { cmd('get_map').catch(() => {}); cmd('get_logs').catch(() => {}); cmd('get_logs', {}, '/plant/cmd').catch(() => {}); }, 300);
  };
  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.op !== 'publish' || !m.msg) return;
    let d;
    try { d = JSON.parse(m.msg.data); } catch { return; }
    if (m.topic === '/amr/state') onState(d);
    else if (m.topic === '/amr/map') onMap(d);
    else if (m.topic === '/amr/reply') onReply(d);
    else if (m.topic === '/amr/log') appendLog(d);
    else if (m.topic === '/plant/state') onPlant(d);
    else if (m.topic === '/plant/reply') onReply(d);
    else if (m.topic === '/plant/log') appendLog({ ...d, msg: '[plant] ' + d.msg });
  };
  ws.onclose = () => {
    S.connected = false;
    $('#offline').hidden = false;
    for (const [, p] of S.pending) p.reject(new Error('Disconnected'));
    S.pending.clear();
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
}
function send(o) { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(o)); }
function cmd(name, extra = {}, topic = '/amr/cmd') {
  return new Promise((resolve, reject) => {
    if (!S.connected) { reject(new Error('Not connected')); return; }
    const id = `a${++S.seq}`;
    const timer = setTimeout(() => { S.pending.delete(id); reject(new Error(`${name}: no answer`)); }, 15000);
    S.pending.set(id, { resolve: (r) => { clearTimeout(timer); resolve(r); }, reject: (e) => { clearTimeout(timer); reject(e); } });
    send({ op: 'publish', topic, msg: { data: JSON.stringify({ id, cmd: name, ...extra }) } });
  });
}
function onReply(r) { const p = S.pending.get(r.id); if (p) { S.pending.delete(r.id); p.resolve(r); } }
// while production runs with automatic logistics the plant drives the LD-60: manual jobs would steal the
// pallet from the cells (safety commands stay available)
const plantDrivesAmr = () => S.plant?.status === 'RUNNING' && S.plant?.auto;
const AMR_SAFETY = ['pause', 'resume', 'stop', 'estop', 'reset', 'set_speed'];
async function run(name, extra = {}, quiet = false, topic = '/amr/cmd') {
  if (topic === '/amr/cmd' && plantDrivesAmr() && !AMR_SAFETY.includes(name)) {
    toast('Production is running with automatic AMR logistics - stop production or switch logistics to manual first', true);
    return { ok: false };
  }
  try {
    const r = await cmd(name, extra, topic);
    if (!r.ok) toast(r.msg || `${name} failed`, true); else if (!quiet && r.msg) toast(r.msg);
    return r;
  } catch (e) { toast(e.message, true); return { ok: false }; }
}
let toastTimer;
function toast(msg, err = false) {
  const t = $('#toast');
  t.textContent = msg; t.className = 'toast' + (err ? ' err' : ''); t.hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { t.hidden = true; }, err ? 5000 : 2200);
}

/* ================================================================ map */
const L = {};   // svg layers
function onMap(m) {
  S.map = m;
  const svg = $('#map');
  svg.innerHTML = '';
  // draw order: robot, then the pallet riding on it, then the boxes on the pallet
  for (const name of ['base', 'zones', 'cells', 'route', 'objects', 'plan', 'goals', 'dynamic', 'palletLayer', 'boxes']) L[name] = svgEl('g', {}, svg);
  S.cellsDrawn = false;
  const lay = m.layout || {};
  const h = lay.hall;
  if (h) {
    svgEl('rect', { x: h.xmin, y: -h.ymax, width: h.xmax - h.xmin, height: h.ymax - h.ymin, class: 'm-hall' }, L.base);
    // grid every metre
    for (let x = Math.ceil(h.xmin); x <= h.xmax; x += 1) svgEl('line', { x1: x, y1: -h.ymax, x2: x, y2: -h.ymin, stroke: '#1f2a36', 'stroke-width': 0.015 }, L.base);
    for (let y = Math.ceil(h.ymin); y <= h.ymax; y += 1) svgEl('line', { x1: h.xmin, y1: -y, x2: h.xmax, y2: -y, stroke: '#1f2a36', 'stroke-width': 0.015 }, L.base);
    if (lay.door) svgEl('line', { x1: lay.door.x, y1: -lay.door.ymax, x2: lay.door.x, y2: -lay.door.ymin, stroke: '#e0a400', 'stroke-width': 0.14 }, L.base);
  }
  for (const z of lay.zones || []) {
    svgEl('rect', { x: z.xmin, y: -z.ymax, width: z.xmax - z.xmin, height: z.ymax - z.ymin, fill: z.color, class: 'm-zone' }, L.zones);
    const t = svgEl('text', { x: z.xmin + 0.12, y: -z.ymax + 0.38, class: 'm-zone-label' }, L.zones);
    t.textContent = z.name;
  }
  const t = lay.truck;
  if (t) {
    svgEl('rect', { x: t.xmin, y: -t.ymax, width: t.xmax - t.xmin, height: t.ymax - t.ymin, class: 'm-truck' }, L.objects);
    svgEl('rect', { x: t.xmax, y: -t.ymax + 0.05, width: t.cab_xmax - t.xmax, height: t.ymax - t.ymin - 0.1, rx: 0.2, class: 'm-cab' }, L.objects);
    const tl = svgEl('text', { x: t.xmin + 2.5, y: 0.12, fill: '#26323f', 'font-size': 0.45, 'font-weight': 700 }, L.objects);
    tl.textContent = 'TRUCK';
  }
  for (const [name, st] of Object.entries(m.stations || {})) {
    const g = svgEl('g', { transform: `translate(${st.x} ${-st.y}) rotate(${-st.yaw})` }, L.objects);
    svgEl('rect', { x: -0.5, y: -0.24, width: 1.0, height: 0.48, class: 'm-station' }, g);
    for (let i = -0.45; i <= 0.46; i += 0.075) svgEl('line', { x1: i, y1: -0.22, x2: i, y2: 0.22, stroke: '#7a8a9a', 'stroke-width': 0.012 }, g);
    svgEl('rect', { x: -0.52, y: -0.24, width: 0.03, height: 0.48, fill: '#d93a3a' }, g);
  }
  if (lay.charger) svgEl('rect', { x: lay.charger.x - 0.2, y: -lay.charger.y - 0.06, width: 0.4, height: 0.12, class: 'm-charger' }, L.objects);

  const pts = { ...m.nodes, ...m.goals };
  for (const [a, b] of m.edges) {
    const pa = pts[a], pb = pts[b];
    if (pa && pb) svgEl('line', { x1: pa.x, y1: -pa.y, x2: pb.x, y2: -pb.y, class: 'm-edge' }, L.route);
  }
  for (const n of Object.values(m.nodes || {})) svgEl('circle', { cx: n.x, cy: -n.y, r: 0.07, class: 'm-node' }, L.route);
  for (const [name, gl] of Object.entries(m.goals || {})) {
    const g = svgEl('g', { class: 'm-goal', 'data-goal': name }, L.goals);
    svgEl('circle', { cx: gl.x, cy: -gl.y, r: 0.16 }, g);
    if (gl.yaw != null) {
      const a = (gl.yaw * Math.PI) / 180;
      svgEl('line', { x1: gl.x, y1: -gl.y, x2: gl.x + Math.cos(a) * 0.4, y2: -gl.y - Math.sin(a) * 0.4, stroke: '#79b8ff', 'stroke-width': 0.05 }, g);
    }
    const lbl = svgEl('text', { x: gl.x + 0.22, y: -gl.y - 0.25 }, g);
    lbl.textContent = gl.label || name;
    g.addEventListener('click', (ev) => { ev.stopPropagation(); openGoalMenu(name, ev); });
  }
  L.planLine = svgEl('polyline', { class: 'm-plan', points: '' }, L.plan);
  L.pallet = svgEl('rect', { x: -0.3, y: -0.225, width: 0.6, height: 0.45, class: 'm-pallet' }, L.palletLayer);
  const rb = svgEl('g', { class: 'm-robot' }, L.dynamic);
  const rl = m.robot?.length || 0.699, rw = m.robot?.width || 0.5;
  svgEl('rect', { class: 'body', x: -rl / 2, y: -rw / 2, width: rl, height: rw, rx: 0.06 }, rb);
  svgEl('rect', { class: 'deck', x: -0.34, y: -0.24, width: 0.68, height: 0.48 }, rb);
  svgEl('path', { d: `M ${rl / 2 - 0.02} 0 L ${rl / 2 - 0.2} -0.12 L ${rl / 2 - 0.2} 0.12 Z` }, rb);
  L.robot = rb;
  L.palletOnTop = null;
  buildGoalButtons();
  if (S.plant) onPlant(S.plant);
  if (!S.view) fitView();
  applyView();
  if (S.state) onState(S.state);
}

function fitView() {
  const h = S.map?.layout?.hall || { xmin: -6, xmax: 12, ymin: -8, ymax: 8 };
  const t = S.map?.layout?.truck;
  const xmax = t ? t.cab_xmax : h.xmax;
  S.view = { x: h.xmin - 0.5, y: -h.ymax - 0.5, w: xmax - h.xmin + 1, h: h.ymax - h.ymin + 1 };
  applyView();
}
function cellsView() {                          // palletizing cells + station, close up
  S.view = { x: -2.2, y: -4.6, w: 4.4, h: 9.2 };
  applyView();
}
function applyView() {
  const v = S.view;
  if (v) $('#map').setAttribute('viewBox', `${v.x} ${v.y} ${v.w} ${v.h}`);
}
function svgPoint(ev) {
  const svg = $('#map');
  const r = svg.getBoundingClientRect();
  const v = S.view;
  // preserveAspectRatio meet: compute the actual scale and offsets
  const s = Math.min(r.width / v.w, r.height / v.h);
  const ox = (r.width - v.w * s) / 2, oy = (r.height - v.h * s) / 2;
  return { x: v.x + (ev.clientX - r.left - ox) / s, y: v.y + (ev.clientY - r.top - oy) / s, s };
}
function setupPanZoom() {
  const svg = $('#map');
  svg.addEventListener('wheel', (ev) => {
    ev.preventDefault();
    if (!S.view) return;
    const p = svgPoint(ev);
    const k = ev.deltaY > 0 ? 1.15 : 1 / 1.15;
    const v = S.view;
    const nw = Math.min(80, Math.max(2, v.w * k));
    const kk = nw / v.w;
    S.view = { x: p.x - (p.x - v.x) * kk, y: p.y - (p.y - v.y) * kk, w: v.w * kk, h: v.h * kk };
    applyView();
  }, { passive: false });
  let drag = null;
  svg.addEventListener('mousedown', (ev) => {
    if (ev.button !== 0 || !S.view) return;
    drag = { cx: ev.clientX, cy: ev.clientY, v: { ...S.view }, s: svgPoint(ev).s };
    svg.classList.add('dragging');
  });
  window.addEventListener('mousemove', (ev) => {
    if (!drag) return;
    S.view = { ...drag.v, x: drag.v.x - (ev.clientX - drag.cx) / drag.s, y: drag.v.y - (ev.clientY - drag.cy) / drag.s };
    if (Math.abs(ev.clientX - drag.cx) + Math.abs(ev.clientY - drag.cy) > 4) setFollow(false);
    applyView();
  });
  window.addEventListener('mouseup', () => { drag = null; svg.classList.remove('dragging'); });
  svg.addEventListener('click', () => { $('#goalMenu').hidden = true; });
}
function setFollow(on) {
  S.follow = on;
  $('#btnFollow').classList.toggle('btn-accent', on);
}

function openGoalMenu(name, ev) {
  const menu = $('#goalMenu');
  const wrap = $('.mapwrap').getBoundingClientRect();
  menu.innerHTML = '';
  const gl = S.map.goals[name];
  const title = document.createElement('div');
  title.className = 'muted'; title.style.cssText = 'font-size:12px;padding:2px 4px 6px';
  title.textContent = `${gl.label || name}  (${fmt(gl.x)}, ${fmt(gl.y)}, ${gl.yaw ?? '-'}°)`;
  menu.append(title);
  const add = (label, fn) => { const b = document.createElement('button'); b.className = 'btn btn-sm'; b.textContent = label; b.onclick = () => { menu.hidden = true; fn(); }; menu.append(b); };
  add(`➜ Go to ${name}`, () => run('goto', { goal: name }));
  if (gl.station) {
    add('➜ Go there and load pallet', async () => { const r = await run('goto', { goal: name }, true); if (r.ok) waitIdleThen(() => run('load')); });
    add('➜ Go there and unload pallet', async () => { const r = await run('goto', { goal: name }, true); if (r.ok) waitIdleThen(() => run('unload')); });
  }
  if (name === 'Home') add('⚡ Go home and charge', () => run('charge'));
  menu.style.left = `${ev.clientX - wrap.left + 8}px`;
  menu.style.top = `${ev.clientY - wrap.top + 8}px`;
  menu.hidden = false;
}
function waitIdleThen(fn) {
  const t0 = Date.now();
  const tick = () => {
    const st = S.state?.status;
    if (st === 'IDLE' && Date.now() - t0 > 600) { fn(); return; }
    if (['ERROR', 'ESTOP'].includes(st) || Date.now() - t0 > 180000) return;
    setTimeout(tick, 300);
  };
  setTimeout(tick, 600);
}

function buildGoalButtons() {
  const box = $('#goalButtons');
  box.innerHTML = '';
  for (const [name, gl] of Object.entries(S.map.goals || {})) {
    const b = document.createElement('button');
    b.className = 'btn';
    b.innerHTML = `<span>➜ ${gl.label || name}</span><small>${fmt(gl.x)}, ${fmt(gl.y)}</small>`;
    b.onclick = () => run('goto', { goal: name });
    box.append(b);
  }
}

/* ================================================================ state */
function onState(st) {
  S.state = st;
  const s = st.status;
  $('#statusPill').textContent = st.paused ? 'PAUSED' : s;
  $('#statusPill').className = `pill pill-${st.paused ? 'PAUSED' : s}`;
  $('#taskText').textContent = st.task || (s === 'IDLE' ? 'Idle' : '');
  if (st.pose) $('#kPose').textContent = `${fmt(st.pose[0])}, ${fmt(st.pose[1])}, ${fmt(st.pose[2], 1)}°`;
  $('#kSpeed').textContent = `${fmt(Math.abs(st.v))} m/s`;
  $('#kBatt').textContent = `${fmt(st.battery, 1)}%${st.charging ? ' ⚡' : ''}`;
  const bb = $('#battBar');
  bb.style.width = `${Math.max(0, Math.min(100, st.battery))}%`;
  bb.style.background = st.battery < 20 ? 'var(--red)' : st.battery < 40 ? 'var(--amber)' : 'var(--green)';
  $('#kPayload').textContent = st.payload ? `Pallet ${st.locked ? '🔒 locked' : '(unlocked)'}` : 'empty';
  $('#kGoal').textContent = st.at_goal ? `at ${st.at_goal}` : st.target ? `→ ${st.target}` : 'on route';
  $('#kMissions').textContent = st.missions_done;
  $('#kCycle').textContent = st.cycle_elapsed != null ? `${fmt(st.cycle_elapsed, 0)} s…` : st.last_cycle != null ? `${fmt(st.last_cycle, 1)} s` : '—';
  $('#kOdo').textContent = `${fmt(st.odometer, 1)} m`;
  if (!S.speedDragging) { $('#speed').value = st.speed_limit; $('#speedVal').textContent = `${fmt(st.speed_limit)} m/s`; }

  const err = $('#errorBar');
  if ((s === 'ERROR' || s === 'ESTOP') && st.error) { err.textContent = `⚠ AMR: ${st.error}`; err.hidden = false; }
  else if (S.plant && S.plant.status === 'ERROR' && S.plant.error) { err.textContent = `⚠ ${S.plant.error}`; err.hidden = false; }
  else err.hidden = true;

  const idle = S.connected && s === 'IDLE' && !plantDrivesAmr();
  const busy = ['MOVING', 'DOCKING', 'TRANSFER', 'WAITING', 'CHARGING', 'PAUSED'].includes(s) || st.paused;
  $('#btnPause').disabled = !busy;
  $('#btnPause').textContent = st.paused ? '▶ Resume' : '⏸ Pause';
  $('#btnStop').disabled = !(busy || (st.queue && st.queue.length));
  $('#btnReset').disabled = !['ERROR', 'ESTOP'].includes(s);
  for (const b of document.querySelectorAll('#goalButtons .btn')) b.disabled = !idle;
  const atStation = st.at_goal && S.map?.goals?.[st.at_goal]?.station;
  $('#btnLoad').disabled = !(idle && atStation && !st.payload);
  $('#btnUnload').disabled = !(idle && atStation && st.payload);
  $('#btnCharge').disabled = !idle;
  $('#btnMission').disabled = !idle;

  $('#roStatus').textContent =
    `status   ${s}${st.paused ? ' (paused)' : ''}\ntask     ${st.task || '-'}\nat goal  ${st.at_goal || '-'}\ntarget   ${st.target || '-'}\n` +
    `v / w    ${fmt(st.v)} m/s  ${fmt(st.w, 1)} °/s\nrollers  ${st.rollers ? 'RUNNING' : 'off'}\nlock     ${st.locked ? 'locked' : 'open'}\n` +
    `mission  ${st.mission || '-'}${st.loop ? ' (loop)' : ''}`;
  const q = $('#queueList');
  q.innerHTML = '';
  const items = (st.task ? [st.task] : []).concat(st.queue || []);
  if (!items.length) q.innerHTML = '<li class="muted">empty</li>';
  items.forEach((t, i) => { const li = document.createElement('li'); li.textContent = t; if (i === 0 && st.task) li.className = 'cur'; q.append(li); });

  // map
  if (L.robot && st.pose) {
    L.robot.setAttribute('transform', `translate(${st.pose[0]} ${-st.pose[1]}) rotate(${-st.pose[2]})`);
    if (S.follow && S.view) { S.view = { ...S.view, x: st.pose[0] - S.view.w / 2, y: -st.pose[1] - S.view.h / 2 }; applyView(); }
  }
  if (L.pallet && st.pallet) L.pallet.setAttribute('transform', `translate(${st.pallet[0]} ${-st.pallet[1]}) rotate(${-st.pallet[3]})`);
  if (L.planLine) {
    const pts = (st.plan || []).map((p) => `${p[0]},${-p[1]}`);
    if (st.pose && pts.length) pts[0] = `${st.pose[0]},${-st.pose[1]}`;
    L.planLine.setAttribute('points', pts.join(' '));
  }
}

/* ================================================================ log */
function appendLog(e) {
  const list = $('#logList');
  const atBottom = list.parentElement.scrollTop + list.parentElement.clientHeight >= list.parentElement.scrollHeight - 30;
  const d = document.createElement('div');
  d.className = `logline ${e.level}`;
  d.innerHTML = `<span class="t"></span>`;
  d.firstChild.textContent = e.t;
  d.append(e.msg);
  list.append(d);
  while (list.children.length > 400) list.firstChild.remove();
  if (atBottom) list.parentElement.scrollTop = list.parentElement.scrollHeight;
}

/* ================================================================ wiring */
function wire() {
  $('#btnPause').onclick = () => run(S.state?.paused ? 'resume' : 'pause', {}, true);
  $('#btnStop').onclick = () => run('stop', {}, true);
  $('#btnReset').onclick = () => run('reset', {}, true);
  $('#btnEstop').onclick = () => run('estop', {}, true);
  $('#btnLoad').onclick = () => run('load');
  $('#btnUnload').onclick = () => run('unload');
  $('#btnCharge').onclick = () => run('charge');
  $('#btnMission').onclick = () => run('mission', { name: 'pallet_cycle', loop: $('#chkLoop').checked });
  $('#btnFit').onclick = () => { setFollow(false); fitView(); };
  $('#btnCells').onclick = () => { setFollow(false); cellsView(); };
  $('#btnFollow').onclick = () => {
    setFollow(!S.follow);
    if (S.follow && S.view && S.view.w > 8) { const k = 8 / S.view.w; S.view = { ...S.view, w: 8, h: S.view.h * k }; }
  };
  const sp = $('#speed');
  sp.addEventListener('input', () => { S.speedDragging = true; $('#speedVal').textContent = `${Number(sp.value).toFixed(2)} m/s`; });
  sp.addEventListener('change', async () => { await run('set_speed', { value: Number(sp.value) }, true); S.speedDragging = false; });
  document.querySelectorAll('#tabs button').forEach((b) => {
    b.onclick = () => {
      document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('active', x === b));
      document.querySelectorAll('.tab-body').forEach((x) => { x.hidden = x.id !== `tab-${b.dataset.tab}`; });
    };
  });
  setupPanZoom();
  wirePlant();
}

wire();
connect();

/* ================================================================ plant */
const prun = (name, extra = {}, quiet = false) => run(name, extra, quiet, '/plant/cmd');
function drawCells(st) {
  if (!L.cells || S.cellsDrawn) return;
  S.cellsDrawn = true;
  L.cells.innerHTML = '';
  L.armMarks = {};
  for (const [name, ln] of Object.entries(st.lanes)) {
    const [[sx, sy], [ex, ey]] = ln.conveyor;
    const len = Math.hypot(ex - sx, ey - sy);
    const ang = (Math.atan2(ey - sy, ex - sx) * 180) / Math.PI;
    const cx = (sx + ex) / 2, cy = (sy + ey) / 2;
    const g = svgEl('g', { transform: `translate(${cx} ${-cy}) rotate(${-ang})` }, L.cells);
    svgEl('rect', { x: -len / 2, y: -0.15, width: len, height: 0.30, class: 'm-conv' }, g);
    for (let i = -len / 2 + 0.1; i < len / 2; i += 0.25) svgEl('line', { x1: i, y1: -0.13, x2: i, y2: 0.13, stroke: '#2a3542', 'stroke-width': 0.012 }, g);
    svgEl('line', { x1: -len / 2, y1: -0.15, x2: len / 2, y2: -0.15, stroke: ln.color, 'stroke-width': 0.03 }, g);
    svgEl('line', { x1: -len / 2, y1: 0.15, x2: len / 2, y2: 0.15, stroke: ln.color, 'stroke-width': 0.03 }, g);
    svgEl('rect', { x: len / 2, y: -0.17, width: 0.03, height: 0.34, fill: '#d93a3a' }, g);
    svgEl('rect', { x: -len / 2 - 0.85, y: -0.45, width: 0.8, height: 0.9, rx: 0.05, class: 'm-mach' }, g);
    svgEl('rect', { x: -len / 2 - 0.85, y: -0.45, width: 0.8, height: 0.12, fill: ln.color }, g);
    const [ax, ay] = ln.arm;
    svgEl('circle', { cx: ax, cy: -ay, r: 1.3, fill: 'none', stroke: '#39424d', 'stroke-width': 0.015, 'stroke-dasharray': '0.06 0.06' }, L.cells);
    const ped = svgEl('circle', { cx: ax, cy: -ay, r: 0.16, class: 'm-ped', stroke: ln.color }, L.cells);
    const lbl = svgEl('text', { x: ax + 0.2, y: -ay + 0.08, fill: '#dfe7f0', 'font-size': 0.24, 'font-weight': 700, 'paint-order': 'stroke', stroke: '#0b0f14', 'stroke-width': 0.05 }, L.cells);
    lbl.textContent = name;
    L.armMarks[name] = ped;
  }
  L.zoneMark = svgEl('rect', { x: -0.36, y: -0.29, width: 0.72, height: 0.58, class: 'm-zone-owner', visibility: 'hidden' }, L.cells);
  const t = st.truck;
  if (t) {                                       // truck robot + its stacking area
    const [ax, ay] = t.arm;
    svgEl('rect', { x: ax + 0.42, y: -0.7, width: 0.5, height: 1.4, fill: 'none', stroke: '#5b4a2a', 'stroke-width': 0.015, 'stroke-dasharray': '0.05 0.05' }, L.cells);
    svgEl('circle', { cx: ax, cy: -ay, r: 1.3, fill: 'none', stroke: '#39424d', 'stroke-width': 0.015, 'stroke-dasharray': '0.06 0.06' }, L.cells);
    L.armMarks[t.name] = svgEl('circle', { cx: ax, cy: -ay, r: 0.16, class: 'm-ped', stroke: t.color }, L.cells);
    const lbl = svgEl('text', { x: ax - 0.1, y: -ay - 0.25, fill: '#dfe7f0', 'font-size': 0.24, 'font-weight': 700, 'paint-order': 'stroke', stroke: '#0b0f14', 'stroke-width': 0.05 }, L.cells);
    lbl.textContent = t.name;
  }
}
function onPlant(st) {
  S.plant = st;
  const pill = $('#plantPill');
  pill.textContent = st.status; pill.className = `pill pill-${st.status}`;
  $('#pFill').textContent = `${st.pallet_fill}/${st.pallet_target}`;
  $('#fillBar').style.width = `${(100 * st.pallet_fill) / Math.max(1, st.pallet_target)}%`;
  $('#fillBar').style.background = st.pallet_fill >= st.dispatch_at ? 'var(--amber)' : 'var(--green)';
  $('#pBoxes').textContent = `${st.boxes_made} / ${st.boxes_palletized}`;
  $('#pPallets').textContent = st.pallets_delivered;
  $('#pThru').textContent = st.throughput_h != null ? `${st.throughput_h} boxes/h` : '—';
  const cyc = st.pallet_cycles || [];
  $('#pCycle').textContent = cyc.length ? `${fmt(cyc[cyc.length - 1], 0)} s (avg ${fmt(cyc.reduce((a, b) => a + b, 0) / cyc.length, 0)} s)` : '—';
  $('#pWait').textContent = `${fmt(st.agv_wait_s, 0)} s / ${fmt(st.arm_wait_pallet_s, 0)} s`;
  $('#pLog').textContent = st.logistics + (st.zone_owner ? ` · zone: ${st.zone_owner}` : '');
  $('#dispAt').textContent = st.dispatch_at;
  if (!S.armDrag) { $('#armSpeed').value = st.arm_speed; $('#armSpeedVal').textContent = `${Math.round(st.arm_speed * 100)}%`; }
  if (!S.rateDrag) { $('#machRate').value = st.machine_period; $('#machRateVal').textContent = `${fmt(st.machine_period, 1)} s`; }
  $('#chkAuto').checked = st.auto;
  $('#btnPStart').disabled = st.status !== 'READY';
  $('#btnPStop').disabled = st.status !== 'RUNNING';
  $('#btnPClear').disabled = st.status === 'RUNNING' || st.status === 'BOOTING';
  $('#btnPReset').disabled = st.status !== 'ERROR';
  if (st.status === 'ERROR' && st.error) { const e = $('#errorBar'); e.textContent = `⚠ ${st.error}`; e.hidden = false; }
  else if (!(S.state && ['ERROR', 'ESTOP'].includes(S.state.status))) $('#errorBar').hidden = true;

  const cards = $('#laneCards');
  cards.innerHTML = '';
  for (const [name, ln] of Object.entries(st.lanes)) {
    const d = document.createElement('div');
    d.className = 'lane'; d.style.borderLeftColor = ln.color;
    d.innerHTML = `<div class="lane-head"><span>${name} · TM12S</span><span class="st st-${ln.status}">${ln.status}</span></div>
      <div class="lane-sub"></div>
      <div class="lane-stats"><span>pallet ${ln.placed}/8</span><span>belt ${ln.on_belt} ${ln.belt ? '▶' : '■'}</span><span>made ${ln.made}</span><span>done ${ln.palletized}</span><span>util ${fmt(ln.util, 0)}%</span></div>`;
    d.querySelector('.lane-sub').textContent = ln.task || '—';
    cards.append(d);
  }
  const t = st.truck;
  if (t) {
    $('#tState').textContent = t.truck_state === 'DOCKED' ? 'docked' : 'departing…';
    $('#tFill').textContent = `${t.boxes}/${t.boxes_max}`;
    $('#tBar').style.width = `${(100 * t.boxes) / Math.max(1, t.boxes_max)}%`;
    $('#tLoaded').textContent = t.boxes_loaded;
    $('#tTrucks').textContent = t.trucks_departed;
    $('#tPph').textContent = st.pallets_h != null ? `${fmt(st.pallets_h, 1)}` : '—';
    $('#tAgv').textContent = `${fmt(st.agv_util, 0)}%`;
    $('#tUtil').textContent = `${fmt(t.util, 0)}%`;
    const lv = Object.values(st.lanes);
    $('#tArms').textContent = `${fmt(lv.reduce((a, l) => a + l.util, 0) / Math.max(1, lv.length), 0)}%`;
    const d = document.createElement('div');
    d.className = 'lane'; d.style.borderLeftColor = t.color;
    d.innerHTML = `<div class="lane-head"><span>${t.name} · TM12S · 4-cup bar</span><span class="st st-${t.status}">${t.status}</span></div>
      <div class="lane-sub"></div>
      <div class="lane-stats"><span>truck ${t.rows}/${t.rows_max} rows</span><span>loaded ${t.boxes_loaded}</span><span>trucks ${t.trucks_departed}</span><span>util ${fmt(t.util, 0)}%</span></div>`;
    d.querySelector('.lane-sub').textContent = t.task || '—';
    $('#truckCard').replaceChildren(d);
  }
  if (!L.cells) return;
  drawCells(st);
  for (const [name, ln] of Object.entries({ ...st.lanes, ...(st.truck ? { [st.truck.name]: st.truck } : {}) })) {
    const mk = L.armMarks?.[name];
    if (mk) mk.setAttribute('fill', ['PICK', 'PLACE', 'MOVING'].includes(ln.status) ? '#1f5f35' : ln.status.startsWith('WAIT') ? '#5a4512' : '#2a333d');
  }
  if (L.zoneMark) L.zoneMark.setAttribute('visibility', st.zone_owner ? 'visible' : 'hidden');
  // boxes (only the ones in the plant; the pool waits off-site)
  const seen = new Set();
  S.boxEls = S.boxEls || {};
  const colors = Object.fromEntries(Object.entries(st.lanes).map(([n, l]) => [n, l.color]));
  for (const [bname, x, y, z, lane] of st.boxes) {
    seen.add(bname);
    let r = S.boxEls[bname];
    if (!r || !r.isConnected) {
      r = svgEl('rect', { width: 0.13, height: 0.10, class: 'm-box', x: -0.065, y: -0.05 }, L.boxes);
      S.boxEls[bname] = r;
    }
    const held = Object.values(st.lanes).some((l) => l.held === bname) || (st.truck?.held || []).includes(bname);
    r.setAttribute('fill', held ? '#ffd54a' : z > 0.95 ? '#c08a52' : '#a8794a');
    r.setAttribute('stroke', colors[lane] || '#3d2a14');
    r.setAttribute('transform', `translate(${x} ${-y})`);
  }
  for (const [bname, r] of Object.entries(S.boxEls)) if (!seen.has(bname)) { r.remove(); delete S.boxEls[bname]; }
}
function wirePlant() {
  $('#btnPStart').onclick = () => prun('start');
  $('#btnPStop').onclick = () => prun('stop');
  $('#btnPClear').onclick = () => prun('clear');
  $('#btnPReset').onclick = () => prun('reset');
  $('#chkAuto').onchange = () => prun('set_auto', { value: $('#chkAuto').checked }, true);
  const sp = $('#armSpeed'), mr = $('#machRate');
  sp.oninput = () => { S.armDrag = true; $('#armSpeedVal').textContent = `${Math.round(sp.value * 100)}%`; };
  sp.onchange = async () => { await prun('set_speed', { value: Number(sp.value) }, true); S.armDrag = false; };
  mr.oninput = () => { S.rateDrag = true; $('#machRateVal').textContent = `${Number(mr.value).toFixed(1)} s`; };
  mr.onchange = async () => { await prun('set_rate', { value: Number(mr.value) }, true); S.rateDrag = false; };
}