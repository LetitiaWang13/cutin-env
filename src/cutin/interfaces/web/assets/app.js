// Replay selects stored snapshots only. It never integrates vehicle dynamics.
const LANE_DASH = [18, 14];
const LANE_COUNT = 3;  // The scene is fixed: three parallel lanes.
export function frameIndex(seconds, hz, count) {
  return Math.max(0, Math.min(count - 1, Math.floor(seconds * hz)));
}
export function laneDashOffset(cameraX, scale, startX) {
  // Pattern is anchored at world x=0, including when the road start is clipped.
  const period = LANE_DASH[0] + LANE_DASH[1];
  return ((cameraX * scale + startX) % period + period) % period;
}
export function sliderHint(value, sliderValue, low, high, step) {
  if (!Number.isFinite(value) || value === sliderValue) return '';
  return `Exact number retained; slider ${low}–${high}, step ${step}.`;
}
export const ROLES = ['ego', 'front', 'cutin'];
const label = role => role === 'cutin' ? 'cut_in' : role;  // Name shown in the page.
export function initialPositionIssues(scenario, vehicle, clearance = 0.1) {
  const issues = [], roles = ROLES;
  const length = vehicle.length_m, width = vehicle.width_m;
  const required = length + clearance;
  roles.forEach(role => {
    const x = scenario[role].x_m;
    if (Number.isFinite(x) && (x < length / 2 || x > scenario.road.length_m - length / 2)) {
      issues.push({paths: [`scenario.${role}.x_m`], message: `${label(role)}: center x ${x.toFixed(3)} m must be within ${(length / 2).toFixed(3)}–${(scenario.road.length_m - length / 2).toFixed(3)} m so the full vehicle stays on the road.`});
    }
  });
  const ego = scenario.ego.x_m, front = scenario.front.x_m;
  if (Number.isFinite(ego) && Number.isFinite(front) && front <= ego) {
    issues.push({paths: ['scenario.ego.x_m', 'scenario.front.x_m'], message: `Front must be ahead of ego: front x ${front.toFixed(3)} m, ego x ${ego.toFixed(3)} m.`});
  }
  roles.forEach((a, index) => roles.slice(index + 1).forEach(b => {
    const first = scenario[a], second = scenario[b];
    const distance = Math.abs(first.x_m - second.x_m);
    if (Number.isFinite(distance) && Math.abs(first.lane - second.lane) * scenario.road.lane_width_m <= width && distance + 1e-9 < required) {
      issues.push({paths: [`scenario.${a}.x_m`, `scenario.${b}.x_m`], message: `${label(a)}/${label(b)}: center separation ${Number(distance.toFixed(9))} m; require at least ${required.toFixed(3)} m (vehicle length ${length.toFixed(3)} m + clearance ${clearance.toFixed(3)} m). Move these vehicles farther apart.`});
    }
  }));
  return issues;
}
export function quantity(value, unit) {
  return value === null ? 'n/a' : `${value.toFixed(3)} ${unit}`;
}
export function validateReplay(data) {
  // Takes the saved {record, metrics}; returns the internal replay {log, metrics, hz}.
  const {record, metrics} = data;
  const hz = record?.simulation?.physics_hz;
  if (!Number.isInteger(hz) || hz <= 0 || metrics?.physics_hz !== hz || !Array.isArray(record.frames)) throw Error('Invalid replay metadata');
  if (!Array.isArray(metrics.pairs) || metrics.termination_reason !== record.termination_reason) throw Error('Invalid recorded metrics');
  metrics.pairs.forEach(pair => {
    if (![pair.min_gap_m, pair.min_ttc_s].every(value => value === null || Number.isFinite(value))) throw Error('Invalid metric values');
  });
  const entries = record.frames.map((frame, index) => {
    if (frame?.tick !== index) throw Error('Invalid replay clock');
    const vehicles = ROLES.map(role => ({...frame[role], role: label(role)}));
    vehicles.forEach(v => {
      if (![v.x_m, v.y_m, v.heading_rad, v.speed_mps, v.length_m, v.width_m].every(Number.isFinite) || v.length_m <= 0 || v.width_m <= 0 || v.speed_mps < 0 || typeof v.collided !== 'boolean') throw Error('Invalid replay geometry');
    });
    return {snapshot: {tick: frame.tick, vehicles, phase: frame.cutin_phase}};
  });
  return {hz, metrics, log: {scenario: record.scenario, entries, termination_reason: record.termination_reason, record}};
}

async function start() {
  const $ = id => document.getElementById(id);
  const fields = [
    ['scenario.seed', 'Scenario seed', 0, null, 1],
    ['scenario.duration_s', 'Horizon (s)', 0.1, 60, 0.1],
    ['scenario.ego.x_m', 'Ego x (m)', 0, null, 'any'],
    ['scenario.ego.speed_mps', 'Ego speed (m/s)', 0, 40, 'any'],
    ['scenario.front.x_m', 'Front x (m)', 0, null, 'any'],
    ['scenario.front.speed_mps', 'Front speed (m/s)', 0, 40, 'any'],
    ['scenario.cutin.x_m', 'Cut-in x (m)', 0, null, 'any'],
    ['scenario.cutin.speed_mps', 'Cut-in speed (m/s)', 0, 40, 'any'],
    ['scenario.cutin_controller.target_gap_m', 'Target gap at merge (m)', null, null, 'any'],
    ['scenario.cutin_controller.lane_change_duration_s', 'Reference duration (s)', 0.01, null, 'any'],
    ['scenario.cutin_controller.warmup_s', 'Warm-up (s)', 0, null, 'any'],
    ['simulation.physics_hz', 'Physics Hz', 1, 120, 1],
    ['simulation.controller_hz', 'Controller Hz', 1, 120, 1],
    ['evaluation.gap_warning_m', 'Gap warning (m)', 0, null, 'any'],
    ['evaluation.ttc_warning_s', 'TTC warning (s)', 0, null, 'any'],
  ];
  const controls = new Map();
  let busy = false;
  let defaults, input, record = null, hz = 30, index = 0, playing = false, last = null, time = 0;
  const get = (object, path) => path.split('.').reduce((value, key) => value[key], object);
  function set(object, path, value) {
    const parts = path.split('.');
    const leaf = parts.pop();
    parts.reduce((target, key) => target[key], object)[leaf] = value;
  }
  async function request(url, options = {}) {
    const response = await fetch(url, options);
    const data = await response.json();
    if (!response.ok) throw Error(data.error || `HTTP ${response.status}`);
    return data;
  }
  function message(text) { $('status').textContent = text; }
  function feedback(text, kind) {
    $('run-feedback').textContent = text;
    $('run-feedback').className = `feedback ${kind}`;
    $('run-feedback').hidden = false;
  }
  function updatePositions() {
    if (!input) return [];
    const scenario = structuredClone(input.scenario);
    ROLES.forEach(role => {
      scenario[role].x_m = controls.get(`scenario.${role}.x_m`).number.valueAsNumber;
      controls.get(`scenario.${role}.x_m`).number.removeAttribute('aria-invalid');
    });
    const issues = initialPositionIssues(scenario, defaults.vehicle);
    $('position-error-list').replaceChildren();
    issues.forEach(issue => {
      const item = document.createElement('li'); item.textContent = issue.message; $('position-error-list').append(item);
      issue.paths.forEach(path => controls.get(path).number.setAttribute('aria-invalid', 'true'));
    });
    $('position-errors').hidden = !issues.length;
    $('run').disabled = busy || !!issues.length;
    return issues;
  }
  function preset(unsafe = false) {
    input = structuredClone(defaults);
    if (unsafe) {
      input.scenario.cutin.x_m = 60;
      input.scenario.cutin.speed_mps = 20;
      input.scenario.cutin_controller.target_gap_m = 3;
      input.scenario.cutin_controller.lane_change_duration_s = 1;
      input.scenario.cutin_controller.warmup_s = 0;
    }
    fields.forEach(([path]) => {
      controls.get(path).number.value = get(input, path);
      controls.get(path).sync();
    });
    updateOffsets();
    $('run-feedback').hidden = true;
    $('go-replay').hidden = true;
  }
  function pause() { playing = false; last = null; $('play').textContent = 'Play'; }
  function draw() {
    const canvas = $('road'), ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!record?.log.entries.length) {
      ctx.fillStyle = '#8b98a5'; ctx.font = '500 15px system-ui, sans-serif'; ctx.textAlign = 'center';
      ctx.fillText(record ? 'This run has no replay samples' : 'Run a scenario to see its replay here', canvas.width / 2, canvas.height / 2 + 5);
      ctx.textAlign = 'start';
      return;
    }
    const snapshot = record.log.entries[index].snapshot;
    const ego = snapshot.vehicles.find(v => v.role === 'ego');
    // Horizontal scale (px per metre) and camera origin depend on the chosen view.
    // Lanes keep a readable height (vscale) even when the view is zoomed far out.
    let scale = 6, originX = (ego?.x_m || 0) - canvas.width * 0.2 / scale;
    const view = $('view').value;
    if (view === 'wide') {
      scale = 3; originX = (ego?.x_m || 0) - canvas.width * 0.15 / scale;
    } else if (view === 'fit') {
      const left = Math.min(...snapshot.vehicles.map(v => v.x_m - v.length_m / 2));
      const right = Math.max(...snapshot.vehicles.map(v => v.x_m + v.length_m / 2));
      scale = Math.max(0.3, Math.min(8, canvas.width * 0.86 / Math.max(right - left, 1)));
      originX = left - canvas.width * 0.07 / scale;
    }
    const vscale = Math.max(scale, 6);
    const road = record.log.scenario.road;
    const laneWidth = road.lane_width_m;
    const x = world => (world - originX) * scale;
    const y = world => canvas.height / 2 + (world - laneWidth) * vscale;
    const startX = Math.max(0, Math.min(canvas.width, x(0)));
    const endX = Math.max(0, Math.min(canvas.width, x(road.length_m)));
    ctx.fillStyle = '#3b434b';
    ctx.fillRect(startX, y(-laneWidth / 2), endX - startX, laneWidth * LANE_COUNT * vscale);
    ctx.strokeStyle = '#edf0ef';
    ctx.lineDashOffset = laneDashOffset(originX, scale, startX);
    for (let lane = 0; lane <= LANE_COUNT; lane++) {
      ctx.setLineDash(lane === 0 || lane === LANE_COUNT ? [] : LANE_DASH);
      ctx.beginPath(); ctx.moveTo(startX, y((lane - 0.5) * laneWidth)); ctx.lineTo(endX, y((lane - 0.5) * laneWidth)); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.lineDashOffset = 0;
    const colors = {ego: '#50b4ff', front: '#64dc82', cut_in: '#ff9632'};
    snapshot.vehicles.forEach(v => {
      // Draw in metres under a (possibly non-uniform) scale, with a minimum on-screen size.
      const length = Math.max(v.length_m, 12 / scale), width = Math.max(v.width_m, 6 / vscale);
      ctx.save(); ctx.translate(x(v.x_m), y(v.y_m)); ctx.scale(scale, vscale); ctx.rotate(v.heading_rad);
      ctx.fillStyle = v.collided ? '#ff5d68' : (colors[v.role] || '#eee');
      ctx.fillRect(-length / 2, -width / 2, length, width);
      ctx.lineWidth = 1 / Math.min(scale, vscale); ctx.strokeStyle = '#101923';
      ctx.strokeRect(-length / 2, -width / 2, length, width);
      ctx.restore();
    });
    ctx.fillStyle = '#fff'; ctx.font = '600 12px system-ui, sans-serif';
    snapshot.vehicles.forEach(v => ctx.fillText(v.role, x(v.x_m) - 16, y(v.y_m) - 16));
    $('seek').value = index;
    $('clock').textContent = `tick ${snapshot.tick} · ${(snapshot.tick / hz).toFixed(3)} s · cut-in ${snapshot.phase}`;
  }
  function mergeText(cutIn) {
    if (cutIn.merge_start_time_s === null) return 'never started (the gap to ego never reached the target gap)';
    return `started at ${cutIn.merge_start_time_s.toFixed(1)} s`;
  }
  function show(data, id) {
    ({hz} = record = validateReplay(data)); index = 0; time = 0; pause();
    $('clock').textContent = 'No samples';
    $('seek').max = Math.max(0, record.log.entries.length - 1);
    $('seek').disabled = $('play').disabled = !record.log.entries.length;
    $('play').classList.toggle('replay-ready', !!record.log.entries.length);
    const m = record.metrics;
    $('assessment').textContent = `${m.assessment_status} · termination: ${record.log.termination_reason} · ego collision: ${m.ego_collision} · gap warning: ${m.gap_warning} · TTC warning: ${m.ttc_warning} · cut-in merge: ${mergeText(m.cut_in)}`;
    const source = record.log.record;
    $('metrics').textContent = JSON.stringify({id, metrics: m, simulation: source.simulation, backend_version: source.backend_version, controller_version: source.controller_version}, null, 2);
    $('pairs').replaceChildren();
    m.pairs.forEach(pair => {
      const row = document.createElement('tr');
      [ `ego/${pair.other_role}`, quantity(pair.min_gap_m, 'm'), pair.min_gap_tick ?? 'n/a', quantity(pair.min_ttc_s, 's'), pair.min_ttc_tick ?? 'n/a' ].forEach(value => {
        const cell = document.createElement('td'); cell.textContent = value; row.append(cell);
      });
      $('pairs').append(row);
    });
    message(`Loaded ${id}. ${record.log.entries.length ? 'Click Play to start recorded replay; no refresh needed.' : 'No replay samples; check termination and diagnostics.'} Complete does not mean safe.`);
    draw();
  }
  // The built-in "default" run is recomputed on load and kept only in memory;
  // saved runs live in the runs folder, one subfolder per name.
  let defaultRecord = null, defaultError = '', sessionRuns = [], nameEdited = false;
  const runUrl = id => `/api/runs/${encodeURIComponent(id)}`;
  // Next "Input N" counts only this page session's runs, so every session starts at Input 1.
  function nextInputName() {
    const used = sessionRuns.map(run => /^input (\d+)$/i.exec(run.name)).filter(Boolean).map(match => Number(match[1]));
    return `Input ${Math.max(0, ...used) + 1}`;
  }
  // The dropdown lists only this session's runs; the runs folder keeps every saved run.
  function refresh(selected = '') {
    const options = (defaultRecord ? [{id: 'default', name: 'default'}] : []).concat(sessionRuns);
    $('saved').replaceChildren(...options.map(run => {
      const option = document.createElement('option'); option.value = run.id; option.textContent = run.name; return option;
    }));
    if (selected) $('saved').value = selected;
    if (!nameEdited) $('run-name').value = nextInputName();
  }
  $('run-name').addEventListener('input', () => { nameEdited = true; });
  async function loadById(id) { show(id === 'default' ? defaultRecord : await request(runUrl(id)), id); }
  $('scenario-form').addEventListener('submit', async event => {
    event.preventDefault();
    const name = $('run-name').value.trim().replace(/\s+/g, ' ') || nextInputName();
    if (name.toLowerCase() === 'default' || sessionRuns.some(run => run.name.toLowerCase() === name.toLowerCase())) { feedback(`The name "${name}" is already used in this session. Pick another name.`, 'error'); return; }
    if (busy || updatePositions().length) return;
    busy = true; $('run').disabled = true; $('run').textContent = 'Running and saving…';
    $('go-replay').hidden = true;
    feedback('Running… wait for the replay-ready confirmation below.', '');
    pause(); message('Computing and saving run…');
    try {
      fields.forEach(([path]) => set(input, path, Number(document.querySelector(`[data-path="${path}"]`).value)));
      const result = await request('/api/runs', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Cutin-Request': '1'}, body: JSON.stringify({...input, name})});
      const saved = await request(runUrl(result.id));
      show(saved, result.id);
      const hasSamples = !!saved.record.frames.length;
      feedback(`Saved "${result.id}" in the runs folder. ${hasSamples ? 'The replay and Metrics above are already loaded. Click Play to begin; no refresh needed.' : 'The saved run has no replay samples. Check its termination and diagnostics above.'}`, 'success');
      $('go-replay').hidden = !hasSamples;
      nameEdited = false;
      sessionRuns.push({id: result.id, name});
      refresh(result.id);
    } catch (error) { feedback(`Run failed: ${error.message}`, 'error'); message(error.message); }
    finally { busy = false; $('run').textContent = 'Run and save'; updatePositions(); }
  });
  $('go-replay').onclick = () => {
    document.querySelector('.replay-panel').scrollIntoView({behavior: 'smooth', block: 'start'});
    $('play').focus({preventScroll: true});
  };
  $('baseline').onclick = () => preset(); $('unsafe').onclick = () => preset(true);
  $('refresh').onclick = () => refresh($('saved').value);
  $('load').onclick = async () => {
    try { const id = $('saved').value; if (id) await loadById(id); }
    catch (error) { message(error.message); }
  };
  // Picking a run in the list loads it; the button stays for keyboard and repeat loads.
  $('saved').addEventListener('change', () => $('load').click());
  $('play').onclick = () => { if (playing) pause(); else { if (index === record.log.entries.length - 1) {index = 0; time = 0;} playing = true; last = null; $('play').textContent = 'Pause'; } };
  $('view').addEventListener('change', draw);
  $('seek').oninput = () => { pause(); index = Number($('seek').value); time = index / hz; draw(); };
  function animate(now) {
    if (playing && record) {
      if (last !== null) time += (now - last) / 1000 * Number($('speed').value);
      last = now; index = frameIndex(time, hz, record.log.entries.length); draw();
      if (index === record.log.entries.length - 1) pause();
    }
    requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);
  function updateOffsets() {
    const value = role => controls.get(`scenario.${role}.x_m`).number.valueAsNumber;
    const offset = role => {
      const delta = value(role) - value('ego');
      return Number.isFinite(delta) ? `${delta >= 0 ? '+' : ''}${delta.toFixed(2)} m` : 'n/a';
    };
    $('offsets').textContent = `Initial center offsets · front − ego: ${offset('front')} · cut-in − ego: ${offset('cutin')}`;
    updatePositions();
  }
  fields.forEach(([path, text, min, max, step], index) => {
    const field = document.createElement('div'); field.className = 'field';
    const label = document.createElement('label'); label.textContent = text; label.htmlFor = `parameter-${index}`;
    const number = document.createElement('input'); number.type = 'number'; number.id = label.htmlFor;
    number.required = true; number.step = step; number.dataset.path = path;
    if (min !== null) number.min = min;
    if (max !== null) number.max = max;
    field.append(label, number);
    let group, quick;
    if (ROLES.includes(path.split('.')[1])) {
      const role = path.split('.')[1];
      group = `${role === 'cutin' ? 'cut-in' : role}-fields`;
      quick = path.endsWith('.x_m') ? [2.5, 200, 0.1] : [0, 40, 0.1];
    } else if (path.startsWith('scenario.cutin_controller.')) {
      group = 'cut-in-maneuver-fields';
      quick = path.endsWith('target_gap_m') ? [0, 60, 0.5] : path.endsWith('warmup_s') ? [0, 10, 0.1] : [0.1, 10, 0.1];
    } else if (path.startsWith('simulation.')) group = 'numerical-fields';
    else if (path.startsWith('evaluation.')) { group = 'evaluation-fields'; quick = [0, 10, 0.1]; }
    else { group = 'experiment-fields'; if (path.endsWith('duration_s')) quick = [0.1, 60, 0.1]; }
    let sync = () => {};
    if (quick) {
      const [low, high, increment] = quick;
      const slider = document.createElement('input'); slider.type = 'range';
      slider.min = low; slider.max = high; slider.step = increment;
      slider.dataset.sliderPath = path; slider.setAttribute('aria-label', `${text} slider`);
      const hint = document.createElement('small'); hint.className = 'precision-hint'; hint.hidden = true;
      sync = () => {
        const value = number.valueAsNumber;
        if (Number.isFinite(value)) slider.value = value;
        hint.textContent = sliderHint(value, Number(slider.value), low, high, increment);
        hint.hidden = !hint.textContent;
      };
      slider.addEventListener('input', () => { number.value = slider.value; sync(); updateOffsets(); });
      field.append(slider, hint);
    }
    number.addEventListener('input', () => { sync(); updateOffsets(); });
    controls.set(path, {number, sync});
    $(group).append(field);
  });
  draw();
  try { defaults = await request('/api/defaults'); preset();
    // The built-in default is computed for display only; ?autorun=0 skips it (used by the smoke script).
    if (new URLSearchParams(location.search).get('autorun') !== '0') {
      try { defaultRecord = await request('/api/runs', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Cutin-Request': '1'}, body: JSON.stringify({...input, save: false})}); }
      catch (error) { defaultError = error.message; }
    }
    refresh();
    if (defaultRecord) { $('saved').value = 'default'; show(defaultRecord, 'default'); }
    else message(defaultError ? `Default run failed: ${defaultError}. Restart "cutin web" so the server matches this page.` : 'Ready. Run a scenario or load a saved run/search winner.');
  }
  catch (error) { message(error.message); $('run').disabled = true; }
}

if (typeof document !== 'undefined') start();
