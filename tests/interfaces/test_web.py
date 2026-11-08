"""Local HTTP boundaries and replay persistence, without browser physics."""

import json
import re
import shutil
import subprocess
from http.client import HTTPConnection
from pathlib import Path
from threading import Event, Thread

import pytest

from cutin.application import run_scenario, save_run
from cutin.core import ScenarioConfig, SimulationLog, TerminationReason
from cutin.interfaces.web.server import MAX_BODY, WebServer


def fake_workflow(scenario, simulator, evaluation):
    def failed(actual, config):
        return SimulationLog(
            actual,
            (),
            TerminationReason.ERROR,
            diagnostic="synthetic failure",
            simulator_settings=config.to_dict(),
        )

    return run_scenario(scenario, simulator, evaluation, simulate_fn=failed)


@pytest.fixture
def web(tmp_path):
    server = WebServer(tmp_path, 0, workflow=fake_workflow)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()
    thread.join()


def request(server, method, path, data=None, headers=None):
    connection = HTTPConnection("127.0.0.1", server.server_port, timeout=10)
    body = json.dumps(data) if data is not None else None
    default = (
        {
            "Origin": f"http://127.0.0.1:{server.server_port}",
            "Content-Type": "application/json",
            "X-Cutin-Request": "1",
        }
        if method == "POST"
        else {}
    )
    default.update(headers or {})
    connection.request(method, path, body=body, headers=default)
    response = connection.getresponse()
    content = response.read()
    status, response_headers = response.status, dict(response.getheaders())
    connection.close()
    return status, content, response_headers


def test_assets_defaults_and_completed_run_reload(web):
    for path in ("/", "/app.js", "/style.css"):
        status, content, headers = request(web, "GET", path)
        assert status == 200 and content
        assert headers["X-Content-Type-Options"] == "nosniff"
    status, content, _ = request(web, "GET", "/api/defaults")
    defaults = json.loads(content)
    status, content, _ = request(web, "POST", "/api/runs", defaults)
    assert status == 201
    identifier = json.loads(content)["id"]
    assert re.fullmatch(r"frontend-\d{8}-\d{6}-[0-9a-f]{8}", identifier)
    saved = web.directory / identifier
    assert (saved / "metrics.json").is_file()
    status, content, _ = request(web, "GET", f"/api/runs/{identifier}")
    record = json.loads(content)
    assert status == 200 and record["metrics"]["assessment_status"] == "incomplete"
    assert record["log"]["diagnostic"] == "synthetic failure"
    status, content, _ = request(web, "GET", "/api/runs")
    assert json.loads(content)["ids"] == [identifier]


@pytest.mark.parametrize(
    "headers",
    [
        {"Host": "evil.example"},
        {"Origin": "https://evil.example"},
        {"Origin": "null"},
        {"X-Cutin-Request": "0"},
    ],
)
def test_mutations_reject_untrusted_origin_host_or_missing_header(web, headers):
    status, _, _ = request(
        web, "POST", "/api/runs", {"scenario": ScenarioConfig().to_dict()}, headers
    )
    assert status == 403
    assert not list(web.directory.iterdir())


@pytest.mark.parametrize(
    "payload",
    [
        [],
        {},
        {"scenario": {}, "unknown": True},
        {"scenario": {"duration_s": 61}},
        {"scenario": {}, "simulator": {"physics_hz": 150}},
        {"scenario": {}, "evaluation": {"ttc_warning_s": -1}},
        {"scenario": {}, "simulator": {"unknown": 1}},
    ],
)
def test_bad_input_and_resource_limits_do_not_run_or_write(web, payload):
    def never(*args):
        pytest.fail("invalid inputs must not call the workflow")

    web.workflow = never
    assert request(web, "POST", "/api/runs", payload)[0] == 400
    assert not list(web.directory.iterdir())


def test_corrupt_or_oversized_saved_artifact_is_rejected(web):
    from cutin.interfaces.web.server import MAX_ARTIFACT

    directory = web.directory / "corrupt"
    directory.mkdir()
    (directory / "log.json").write_text("not json")
    (directory / "metrics.json").write_text("{}")
    assert request(web, "GET", "/api/runs/corrupt")[0] == 400
    with (directory / "log.json").open("w") as file:
        file.truncate(MAX_ARTIFACT + 1)
    assert request(web, "GET", "/api/runs/corrupt")[0] == 400


def test_body_limit_and_content_type(web):
    assert (
        request(web, "POST", "/api/runs", {}, {"Content-Type": "text/plain"})[0] == 415
    )
    assert (
        request(web, "POST", "/api/runs", {}, {"Content-Length": str(MAX_BODY + 1)})[0]
        == 413
    )
    assert (
        request(web, "POST", "/api/runs", {}, {"Transfer-Encoding": "chunked"})[0]
        == 400
    )


def test_busy_run_is_rejected_without_queueing(web):
    started, release = Event(), Event()

    def blocking(*args):
        started.set()
        assert release.wait(5)
        return fake_workflow(*args)

    web.workflow = blocking
    responses = []
    thread = Thread(
        target=lambda: responses.append(
            request(web, "POST", "/api/runs", {"scenario": {}})
        )
    )
    thread.start()
    assert started.wait(5)
    try:
        assert request(web, "POST", "/api/runs", {"scenario": {}})[0] == 409
    finally:
        release.set()
        thread.join()
    assert responses[0][0] == 201


def test_existing_cli_run_and_search_best_can_be_loaded_without_simulation(web):
    from cutin.evaluation import EvaluationConfig
    from cutin.simulation import SimulatorConfig

    result = fake_workflow(ScenarioConfig(), SimulatorConfig(), EvaluationConfig())
    save_run(result, web.directory / "cli-run")
    save_run(result, web.directory / "search-result" / "best")

    def never(*args):
        pytest.fail("replay must not invoke simulation")

    web.workflow = never
    for identifier in ("cli-run", "search-result"):
        assert request(web, "GET", f"/api/runs/{identifier}")[0] == 200


def test_traversal_and_symlink_escape_do_not_read_outside_root(web, tmp_path):
    outside = tmp_path.parent / f"{tmp_path.name}-outside"
    outside.mkdir()
    (outside / "log.json").write_text("{}")
    (outside / "metrics.json").write_text("{}")
    (web.directory / "escape").symlink_to(outside, target_is_directory=True)
    assert request(web, "GET", "/api/runs/escape")[0] == 400
    assert request(web, "GET", "/api/runs/../outside")[0] == 400
    assert request(web, "GET", "/api/runs/missing")[0] == 404
    assert json.loads(request(web, "GET", "/api/runs")[1])["ids"] == []


def test_real_unsafe_request_uses_existing_workflow(web):
    web.workflow = run_scenario
    scenario = ScenarioConfig().to_dict()
    scenario["initial"]["cut_in"]["x_m"] = 50
    scenario["cut_in"].update(start_time_s=0, lane_change_duration_s=1)
    status, content, _ = request(web, "POST", "/api/runs", {"scenario": scenario})
    assert status == 201
    result = json.loads(content)
    assert result["metrics"]["ego_collision"]
    record = json.loads(request(web, "GET", f"/api/runs/{result['id']}")[1])
    assert record["log"]["entries"][-1]["snapshot"]["tick"] == 18


def test_replay_js_selects_samples_and_preserves_null_semantics():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is optional; use browser smoke checks for replay JavaScript")
    asset = (
        Path(__file__).resolve().parents[2] / "src/cutin/interfaces/web/assets/app.js"
    )
    code = """
import fs from 'node:fs'; import assert from 'node:assert/strict';
const text = fs.readFileSync(process.argv[1], 'utf8');
const source = 'data:text/javascript;base64,' +
  Buffer.from(text).toString('base64');
const {frameIndex, quantity, validateReplay} = await import(source);
assert.equal(frameIndex(0, 30, 301), 0);
assert.equal(frameIndex(2, 30, 301), 60);
assert.equal(frameIndex(100, 30, 301), 300);
assert.equal(quantity(null, 's'), 'n/a');
const data = {
  log: {simulator_settings: {physics_hz:30},
    entries:[{snapshot:{tick:0, vehicles:[]}}]},
  metrics:{physics_hz:30, pairs:[]}
};
assert.equal(validateReplay(data), 30);
data.log.entries[0].snapshot.tick = 1;
assert.throws(() => validateReplay(data));
"""
    subprocess.run([node, "--input-type=module", "-e", code, str(asset)], check=True)


def test_named_runs_use_timestamp_and_name_as_folder_and_default_is_not_saved(web):
    defaults = json.loads(request(web, "GET", "/api/defaults")[1])
    status, content, _ = request(web, "POST", "/api/runs", {**defaults, "name": " Input  1 "})
    identifier = json.loads(content)["id"]
    assert status == 201 and re.fullmatch(r"\d{8}-\d{6}-Input 1", identifier)
    assert (web.directory / identifier / "log.json").is_file()
    assert json.loads(request(web, "GET", "/api/runs")[1])["ids"] == [identifier]
    assert request(web, "GET", "/api/runs/" + identifier.replace(" ", "%20"))[0] == 200
    for bad in ("", "x" * 41, 7, "a/b", "..", "bad\x00name", "Default"):
        assert request(web, "POST", "/api/runs", {**defaults, "name": bad})[0] == 400
    status, content, _ = request(web, "POST", "/api/runs", {**defaults, "save": False})
    assert status == 200 and {"log", "metrics"} <= set(json.loads(content))
    assert [path.name for path in web.directory.iterdir()] == [identifier]
