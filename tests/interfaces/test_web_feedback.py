"""Web clearance and readable IDs; core/search collision constraints stay unchanged."""

import json
import shutil
import subprocess
from dataclasses import replace
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

import pytest

from cutin.core import ScenarioConfig
from cutin.interfaces.web.server import WebServer
from cutin.interfaces.web.validation import validate_initial_clearance


@pytest.mark.parametrize(
    "front_x, valid", [(54.9, False), (55.05, False), (55.1, True), (55.2, True)]
)
def test_same_lane_web_buffer(front_x, valid):
    base = ScenarioConfig()
    candidate = replace(
        base,
        initial=replace(base.initial, front=replace(base.initial.front, x_m=front_x)),
    )
    if valid:
        validate_initial_clearance(candidate)
    else:
        with pytest.raises(ValueError, match="ego/front.*center separation.*clearance"):
            validate_initial_clearance(candidate)


def test_buffer_does_not_reject_adjacent_lane_same_x_or_decimal_boundary():
    base = ScenarioConfig()
    candidate = replace(
        base, initial=replace(base.initial, cut_in=replace(base.initial.cut_in, x_m=50))
    )
    validate_initial_clearance(candidate)
    boundary = replace(
        base,
        initial=replace(
            base.initial,
            ego=replace(base.initial.ego, x_m=100),
            front=replace(base.initial.front, x_m=105.1),
        ),
    )
    validate_initial_clearance(boundary)


def test_api_returns_specific_clearance_reason_without_starting_run(tmp_path):
    def never(*args):
        pytest.fail("invalid spacing must not invoke physics")

    server = WebServer(tmp_path, 0, workflow=never)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        scenario = ScenarioConfig().to_dict()
        scenario["initial"]["front"]["x_m"] = 55.05
        connection = HTTPConnection("127.0.0.1", server.server_port)
        connection.request(
            "POST",
            "/api/runs",
            json.dumps({"scenario": scenario}),
            {
                "Content-Type": "application/json",
                "Origin": f"http://127.0.0.1:{server.server_port}",
                "X-Cutin-Request": "1",
            },
        )
        response = connection.getresponse()
        assert response.status == 400
        message = json.loads(response.read())["error"]
        assert "ego/front" in message and "5.050" in message and "5.100" in message
        connection.close()
        assert not list(tmp_path.iterdir())
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_position_feedback_js_explains_pairs_and_skips_adjacent_lanes():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node optional; Chromium smoke checks inline validation")
    asset = (
        Path(__file__).resolve().parents[2] / "src/cutin/interfaces/web/assets/app.js"
    )
    code = r"""
import fs from 'node:fs'; import assert from 'node:assert/strict';
const text = fs.readFileSync(process.argv[1], 'utf8');
const source = 'data:text/javascript;base64,' +
  Buffer.from(text).toString('base64');
const {initialPositionIssues} = await import(source);
const scene = {vehicles:{length_m:5, width_m:2},
  road:{length_m:2000, lane_width_m:4},
  initial:{ego:{x_m:50,lane:1}, front:{x_m:55.05,lane:1},
    cut_in:{x_m:50,lane:0}}};
let issues = initialPositionIssues(scene);
assert.equal(issues.length, 1);
assert.match(issues[0].message, /ego\/front.*5.05.*5.100/);
assert.equal(issues[0].paths.length, 2);
scene.initial.front.x_m = 55.1;
assert.equal(initialPositionIssues(scene).length, 0);
scene.initial.ego.x_m = 100; scene.initial.front.x_m = 105.1;
assert.equal(initialPositionIssues(scene).length, 0);
scene.initial.front.x_m = 40;
assert(initialPositionIssues(scene).some(issue => issue.message.includes('ahead')));
scene.initial.ego.x_m = 1;
assert(initialPositionIssues(scene).some(issue =>
  issue.message.includes('full vehicle')));
"""
    subprocess.run([node, "--input-type=module", "-e", code, str(asset)], check=True)
