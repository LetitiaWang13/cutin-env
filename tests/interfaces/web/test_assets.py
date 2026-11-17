"""Browser-side helpers, run in Node when available (Chromium smoke covers the rest)."""

import shutil
import subprocess
from pathlib import Path

import pytest

ASSET = Path(__file__).resolve().parents[3] / "src/cutin/interfaces/web/assets/app.js"
PRELUDE = """
import fs from 'node:fs'; import assert from 'node:assert/strict';
const text = fs.readFileSync(process.argv[1], 'utf8');
const source = 'data:text/javascript;base64,' + Buffer.from(text).toString('base64');
const app = await import(source);
"""


def run_node(body: str) -> None:
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is optional; use the Chromium smoke script instead")
    subprocess.run(
        [node, "--input-type=module", "-e", PRELUDE + body, str(ASSET)], check=True
    )


def test_replay_helpers_select_samples_and_preserve_null_semantics():
    run_node(
        """
const {frameIndex, quantity, validateReplay} = app;
assert.equal(frameIndex(0, 30, 301), 0);
assert.equal(frameIndex(2, 30, 301), 60);
assert.equal(frameIndex(100, 30, 301), 300);
assert.equal(quantity(null, 's'), 'n/a');
const vehicle = x => ({x_m: x, y_m: 4, heading_rad: 0, speed_mps: 25,
  length_m: 5, width_m: 2, collided: false});
const data = {
  record: {simulation: {physics_hz: 30}, termination_reason: 'horizon',
    scenario: {}, frames: [{tick: 0, ego: vehicle(0), front: vehicle(50),
      cutin: vehicle(20), cutin_phase: 'warmup'}]},
  metrics: {physics_hz: 30, termination_reason: 'horizon', pairs: []},
};
const replay = validateReplay(data);
assert.equal(replay.hz, 30);
assert.deepEqual(replay.log.entries[0].snapshot.vehicles.map(v => v.role),
  ['ego', 'front', 'cut_in']);
data.record.frames[0].tick = 1;
assert.throws(() => validateReplay(data));
"""
    )


def test_position_feedback_explains_pairs_and_skips_adjacent_lanes():
    run_node(
        r"""
const {initialPositionIssues} = app;
const vehicle = {length_m: 5, width_m: 2};
const scene = {road: {length_m: 2000, lane_width_m: 4},
  ego: {x_m: 50, lane: 1}, front: {x_m: 55.05, lane: 1}, cutin: {x_m: 50, lane: 0}};
let issues = initialPositionIssues(scene, vehicle);
assert.equal(issues.length, 1);
assert.match(issues[0].message, /ego\/front.*5.05.*5.100/);
assert.deepEqual(issues[0].paths, ['scenario.ego.x_m', 'scenario.front.x_m']);
scene.front.x_m = 55.1;
assert.equal(initialPositionIssues(scene, vehicle).length, 0);
scene.ego.x_m = 100; scene.front.x_m = 105.1;
assert.equal(initialPositionIssues(scene, vehicle).length, 0);
scene.front.x_m = 40;
assert(initialPositionIssues(scene, vehicle).some(i => i.message.includes('ahead')));
scene.ego.x_m = 1;
assert(initialPositionIssues(scene, vehicle)
  .some(i => i.message.includes('full vehicle')));
"""
    )


def test_slider_hints_do_not_transform_numeric_values():
    run_node(
        """
const {sliderHint} = app;
assert.equal(sliderHint(25, 25, 0, 40, 0.1), '');
assert.equal(sliderHint(NaN, 25, 0, 40, 0.1), '');
assert.match(sliderHint(26.425, 26.4, 0, 40, 0.1), /Exact number retained/);
assert.match(sliderHint(1234.125, 200, 2.5, 200, 0.5), /2.5–200/);
"""
    )


def test_lane_phase_tracks_world_motion_and_wraps():
    run_node(
        """
const {laneDashOffset} = app;
const period = 32;
for (const camera of [0, 1.5, 100, -3]) {
  const offset = laneDashOffset(camera, 8, 0);
  assert(offset >= 0 && offset < period);
  assert.equal(laneDashOffset(camera + period / 8, 8, 0), offset);
}
"""
    )
