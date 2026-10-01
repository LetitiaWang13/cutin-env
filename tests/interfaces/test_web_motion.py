"""World-anchored dash phase follows recorded camera positions, not wall time."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_lane_phase_tracks_world_motion_wraps_and_handles_road_start():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is optional; Chromium smoke also checks lane pixels")
    asset = (
        Path(__file__).resolve().parents[2] / "src/cutin/interfaces/web/assets/app.js"
    )
    code = """
import fs from 'node:fs'; import assert from 'node:assert/strict';
const text = fs.readFileSync(process.argv[1], 'utf8');
const source = 'data:text/javascript;base64,' +
  Buffer.from(text).toString('base64');
const {laneDashOffset} = await import(source);
assert.equal(laneDashOffset(18.75, 8, 0), 22);
assert.equal(laneDashOffset(19.75, 8, 0), 30); // +1 m -> 8 px left.
assert.equal(laneDashOffset(22.75, 8, 0), 22); // One full pattern period.
assert.equal(laneDashOffset(-20, 8, 160), 0); // World road start visible.
assert.equal(laneDashOffset(18.75, 8, 0), 22); // Seek is stateless.
assert.equal(laneDashOffset(-1, 8, 0), 24); // Negative modulo normalized.
"""
    subprocess.run([node, "--input-type=module", "-e", code, str(asset)], check=True)
