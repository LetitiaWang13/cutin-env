// Replay selects stored snapshots only. It never integrates vehicle dynamics.
const LANE_DASH = [18, 14];
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
export function quantity(value, unit) {
  return value === null ? 'n/a' : `${value.toFixed(3)} ${unit}`;
}
export function validateReplay(data) {
  const {log, metrics} = data;
  const hz = log?.simulator_settings?.physics_hz;
  if (!Number.isInteger(hz) || hz <= 0 || metrics?.physics_hz !== hz || !Array.isArray(log.entries)) throw Error('Invalid replay metadata');
  if (!Array.isArray(metrics.pairs) || metrics.termination_reason !== log.termination_reason) throw Error('Invalid recorded metrics');
  metrics.pairs.forEach(pair => {
    if (![pair.min_gap_m, pair.min_ttc_s].every(value => value === null || Number.isFinite(value))) throw Error('Invalid metric values');
  });
  log.entries.forEach(({snapshot}, index) => {
    if (snapshot?.tick !== index || !Array.isArray(snapshot.vehicles)) throw Error('Invalid replay clock');
    snapshot.vehicles.forEach(v => {
      if (![v.x_m, v.y_m, v.heading_rad, v.speed_mps, v.length_m, v.width_m].every(Number.isFinite) || v.length_m <= 0 || v.width_m <= 0 || v.speed_mps < 0 || typeof v.collided !== 'boolean' || typeof v.role !== 'string') throw Error('Invalid replay geometry');
    });
  });
  return hz;
}

async function start() {
  const $ = id => document.getElementById(id);
  const fields = [
    ['scenario.seed', 'Scenario seed', 0, null, 1],
    ['scenario.duration_s', 'Horizon (s)', 0.1, 60, 0.1],
    ['scenario.initial.ego.x_m', 'Ego x (m)', 0, null, 'any'],
    ['scenario.initial.ego.speed_mps', 'Ego speed (m/s)', 0, 40, 'any'],
    ['scenario.initial.front.x_m', 'Front x (m)', 0, null, 'any'],
    ['scenario.initial.front.speed_mps', 'Front speed (m/s)', 0, 40, 'any'],
    ['scenario.initial.cut_in.x_m', 'Cut-in x (m)', 0, null, 'any'],
    ['scenario.initial.cut_in.speed_mps', 'Cut-in speed (m/s)', 0, 40, 'any'],
    ['scenario.cut_in.start_time_s', 'Merge start (s)', 0, null, 'any'],
    ['scenario.cut_in.lane_change_duration_s', 'Reference duration (s)', 0.01, null, 'any'],
    ['scenario.cut_in.target_speed_mps', 'Cut-in target (m/s)', 0, 40, 'any'],
    ['simulator.physics_hz', 'Physics Hz', 1, 120, 1],
    ['simulator.controller_hz', 'Controller Hz', 1, 120, 1],
    ['evaluation.gap_warning_m', 'Gap warning (m)', 0, null, 'any'],
    ['evaluation.ttc_warning_s', 'TTC warning (s)', 0, null, 'any'],
  ];
  const controls = new Map();
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
  function preset(unsafe = false) {
    input = structuredClone(defaults);
    if (unsafe) {
      input.scenario.initial.cut_in.x_m = 50;
      input.scenario.cut_in.start_time_s = 0;
      input.scenario.cut_in.lane_change_duration_s = 1;
    }
    fields.forEach(([path]) => {
      controls.get(path).number.value = get(input, path);
      controls.get(path).sync();
    });
    updateOffsets();
  }
  function pause() { playing = false; last = null; $('play').textContent = 'Play'; }
  function draw() {
    const canvas = $('road'), ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!record?.log.entries.length) return;
    const snapshot = record.log.entries[index].snapshot;
    const ego = snapshot.vehicles.find(v => v.role === 'ego');
    const scale = 8, originX = (ego?.x_m || 0) - canvas.width * 0.25 / scale;
    const road = record.log.scenario.road;
    const laneWidth = road.lane_width_m;
    const x = world => (world - originX) * scale;
    const y = world => canvas.height / 2 + (world - laneWidth) * scale;
    const startX = Math.max(0, Math.min(canvas.width, x(0)));
    const endX = Math.max(0, Math.min(canvas.width, x(road.length_m)));
    ctx.fillStyle = '#586067';
    ctx.fillRect(startX, y(-laneWidth / 2), endX - startX, laneWidth * road.lane_count * scale);
    ctx.strokeStyle = '#edf0ef';
    ctx.lineDashOffset = laneDashOffset(originX, scale, startX);
    for (let lane = 0; lane <= road.lane_count; lane++) {
      ctx.setLineDash(lane === 0 || lane === road.lane_count ? [] : LANE_DASH);
      ctx.beginPath(); ctx.moveTo(startX, y((lane - 0.5) * laneWidth)); ctx.lineTo(endX, y((lane - 0.5) * laneWidth)); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.lineDashOffset = 0;
    const colors = {ego: '#50b4ff', front: '#64dc82', cut_in: '#ff9632'};
    snapshot.vehicles.forEach(v => {
      ctx.save(); ctx.translate(x(v.x_m), y(v.y_m)); ctx.rotate(v.heading_rad);
      ctx.fillStyle = v.collided ? '#ff5d68' : (colors[v.role] || '#eee');
      ctx.fillRect(-v.length_m * scale / 2, -v.width_m * scale / 2, v.length_m * scale, v.width_m * scale);
      ctx.strokeStyle = '#101923'; ctx.strokeRect(-v.length_m * scale / 2, -v.width_m * scale / 2, v.length_m * scale, v.width_m * scale);
      ctx.restore();
    });
    ctx.fillStyle = '#fff';
    snapshot.vehicles.forEach(v => ctx.fillText(v.role, x(v.x_m) - 16, y(v.y_m) - 16));
    $('seek').value = index;
    $('clock').textContent = `tick ${snapshot.tick} · ${(snapshot.tick / hz).toFixed(3)} s`;
  }
  function show(data, id) {
    hz = validateReplay(data); record = data; index = 0; time = 0; pause();
    $('clock').textContent = 'No samples';
    $('seek').max = Math.max(0, record.log.entries.length - 1);
    $('seek').disabled = $('play').disabled = !record.log.entries.length;
    const m = record.metrics;
    $('assessment').textContent = `${m.assessment_status} · termination: ${record.log.termination_reason} · ego collision: ${m.ego_collision} · gap warning: ${m.gap_warning} · TTC warning: ${m.ttc_warning}`;
    $('metrics').textContent = JSON.stringify({id, metrics: m, simulator: record.log.simulator_settings, diagnostic: record.log.diagnostic}, null, 2);
    $('pairs').replaceChildren();
    m.pairs.forEach(pair => {
      const row = document.createElement('tr');
      [ `ego/${pair.other_role}`, quantity(pair.min_gap_m, 'm'), pair.min_gap_tick ?? 'n/a', quantity(pair.min_ttc_s, 's'), pair.min_ttc_tick ?? 'n/a' ].forEach(value => {
        const cell = document.createElement('td'); cell.textContent = value; row.append(cell);
      });
      $('pairs').append(row);
    });
    message(`Loaded ${id}. Recorded samples only; complete does not mean safe.`);
    draw();
  }
  async function refresh(selected = '') {
    const {ids} = await request('/api/runs');
    $('saved').replaceChildren(...ids.map(id => {
      const option = document.createElement('option'); option.value = option.textContent = id; return option;
    }));
    if (selected) $('saved').value = selected;
  }
  $('scenario-form').addEventListener('submit', async event => {
    event.preventDefault(); $('run').disabled = true; pause(); message('Computing and saving run…');
    try {
      fields.forEach(([path]) => set(input, path, Number(document.querySelector(`[data-path="${path}"]`).value)));
      const result = await request('/api/runs', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Cutin-Request': '1'}, body: JSON.stringify(input)});
      show(await request(`/api/runs/${result.id}`), result.id); await refresh(result.id);
    } catch (error) { message(error.message); }
    finally { $('run').disabled = false; }
  });
  $('baseline').onclick = () => preset(); $('unsafe').onclick = () => preset(true);
  $('refresh').onclick = () => refresh().catch(error => message(error.message));
  $('load').onclick = async () => {
    try { const id = $('saved').value; if (id) show(await request(`/api/runs/${id}`), id); }
    catch (error) { message(error.message); }
  };
  $('play').onclick = () => { if (playing) pause(); else { if (index === record.log.entries.length - 1) {index = 0; time = 0;} playing = true; last = null; $('play').textContent = 'Pause'; } };
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
    const value = role => controls.get(`scenario.initial.${role}.x_m`).number.valueAsNumber;
    const offset = role => {
      const delta = value(role) - value('ego');
      return Number.isFinite(delta) ? `${delta >= 0 ? '+' : ''}${delta.toFixed(2)} m` : 'n/a';
    };
    $('offsets').textContent = `Initial center offsets · front − ego: ${offset('front')} · cut-in − ego: ${offset('cut_in')}`;
  }
  fields.forEach(([path, text, min, max, step], index) => {
    const field = document.createElement('div'); field.className = 'field';
    const label = document.createElement('label'); label.textContent = text; label.htmlFor = `parameter-${index}`;
    const number = document.createElement('input'); number.type = 'number'; number.id = label.htmlFor;
    number.required = true; number.min = min; number.step = step; number.dataset.path = path;
    if (max !== null) number.max = max;
    field.append(label, number);
    let group, quick;
    if (path.includes('.initial.')) {
      const role = path.split('.')[2];
      group = `${role.replace('_', '-')}-fields`;
      quick = path.endsWith('.x_m') ? [2.5, 200, 0.1] : [0, 40, 0.1];
    } else if (path.startsWith('scenario.cut_in.')) {
      group = 'cut-in-maneuver-fields';
      quick = path.endsWith('target_speed_mps') ? [0, 40, 0.1] : path.endsWith('start_time_s') ? [0, 10, 0.1] : [0.1, 10, 0.1];
    } else if (path.startsWith('simulator.')) group = 'numerical-fields';
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
  try { defaults = await request('/api/defaults'); preset(); await refresh(); message('Ready. Run a scenario or load a saved run/search winner.'); }
  catch (error) { message(error.message); $('run').disabled = true; }
}

if (typeof document !== 'undefined') start();
