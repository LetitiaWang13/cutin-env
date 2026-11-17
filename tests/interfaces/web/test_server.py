"""Local HTTP boundaries, run folders and replay persistence."""

import json
import re
from threading import Event, Thread

import pytest

from cutin.interfaces.web.server import MAX_ARTIFACT, MAX_BODY


def test_assets_and_defaults(web, http):
    for path in ("/", "/app.js", "/style.css"):
        status, content, headers = http(web, "GET", path)
        assert status == 200 and content
        assert headers["X-Content-Type-Options"] == "nosniff"
        assert "default-src 'self'" in headers["Content-Security-Policy"]
    defaults = json.loads(http(web, "GET", "/api/defaults")[1])
    assert set(defaults) == {"scenario", "simulation", "evaluation", "vehicle"}
    assert defaults["scenario"]["cutin_controller"]["target_gap_m"] == 25


def test_unnamed_run_is_saved_and_reloadable(web, defaults, http):
    status, content, _ = http(web, "POST", "/api/runs", defaults)
    identifier = json.loads(content)["id"]
    assert status == 201
    assert re.fullmatch(r"frontend-\d{8}-\d{6}-[0-9a-f]{8}", identifier)
    status, content, _ = http(web, "GET", f"/api/runs/{identifier}")
    saved = json.loads(content)
    assert status == 200 and set(saved) == {"record", "metrics"}
    assert saved["metrics"]["assessment_status"] == "complete"
    assert len(saved["record"]["frames"]) == 3
    assert json.loads(http(web, "GET", "/api/runs")[1])["ids"] == [identifier]


def test_named_runs_use_timestamp_and_name_as_folder(web, defaults, http):
    status, content, _ = http(
        web, "POST", "/api/runs", {**defaults, "name": " Input  1 "}
    )
    identifier = json.loads(content)["id"]
    assert status == 201 and re.fullmatch(r"\d{8}-\d{6}-Input 1", identifier)
    assert (web.directory / identifier / "record.json").is_file()
    assert (web.directory / identifier / "metrics.json").is_file()
    assert http(web, "GET", "/api/runs/" + identifier.replace(" ", "%20"))[0] == 200
    for bad in ("", "x" * 41, 7, "a/b", "..", "bad\x00name", "Default"):
        assert http(web, "POST", "/api/runs", {**defaults, "name": bad})[0] == 400
    assert [path.name for path in web.directory.iterdir()] == [identifier]


def test_default_preview_is_not_saved(web, defaults, http):
    status, content, _ = http(web, "POST", "/api/runs", {**defaults, "save": False})
    assert status == 200 and {"record", "metrics"} == set(json.loads(content))
    assert not list(web.directory.iterdir())


@pytest.mark.parametrize(
    "headers",
    [
        {"Host": "evil.example"},
        {"Origin": "https://evil.example"},
        {"Origin": "null"},
        {"X-Cutin-Request": "0"},
    ],
)
def test_mutations_reject_untrusted_origin_host_or_missing_header(
    web, defaults, headers, http
):
    assert http(web, "POST", "/api/runs", defaults, headers)[0] == 403
    assert not list(web.directory.iterdir())


@pytest.mark.parametrize(
    "mutate",
    [
        lambda d: d.update(unknown=True),
        lambda d: d["scenario"].update(duration_s=61),
        lambda d: d["scenario"].update(unknown=1),
        lambda d: d["scenario"]["front"].update(x_m=45),
        lambda d: d["scenario"]["cutin"].update(lane=1),
        lambda d: d["simulation"].update(physics_hz=150),
        lambda d: d["simulation"].update(physics_hz=30, controller_hz=7),
        lambda d: d["simulation"].update(backend="other"),
        lambda d: d["evaluation"].update(ttc_warning_s=-1),
        lambda d: d["vehicle"].update(length_m=9),
        lambda d: d["scenario"]["cutin_controller"].update(lane_change_duration_s=0),
    ],
)
def test_bad_input_and_resource_limits_do_not_run_or_write(web, defaults, mutate, http):
    def never(*args):
        pytest.fail("invalid inputs must not call the runner")

    web.runner = never
    mutate(defaults)
    assert http(web, "POST", "/api/runs", defaults)[0] == 400
    assert not list(web.directory.iterdir())


@pytest.mark.parametrize("payload", [[], {}, {"scenario": []}])
def test_malformed_payloads_are_rejected(web, payload, http):
    assert http(web, "POST", "/api/runs", payload)[0] == 400


def test_corrupt_or_oversized_saved_artifact_is_rejected(web, http):
    directory = web.directory / "corrupt"
    directory.mkdir()
    (directory / "record.json").write_text("not json")
    (directory / "metrics.json").write_text("{}")
    assert http(web, "GET", "/api/runs/corrupt")[0] == 400
    with (directory / "record.json").open("w") as file:
        file.truncate(MAX_ARTIFACT + 1)
    assert http(web, "GET", "/api/runs/corrupt")[0] == 400


def test_body_limit_and_content_type(web, http):
    assert http(web, "POST", "/api/runs", {}, {"Content-Type": "text/plain"})[0] == 415
    assert (
        http(web, "POST", "/api/runs", {}, {"Content-Length": str(MAX_BODY + 1)})[0]
        == 413
    )
    assert (
        http(web, "POST", "/api/runs", {}, {"Transfer-Encoding": "chunked"})[0] == 400
    )


def test_busy_run_is_rejected_without_queueing(web, defaults, http, make_record):
    started, release = Event(), Event()

    def blocking(*args):
        started.set()
        assert release.wait(5)
        return make_record(*args)

    web.runner = blocking
    responses = []
    thread = Thread(
        target=lambda: responses.append(http(web, "POST", "/api/runs", defaults))
    )
    thread.start()
    assert started.wait(5)
    try:
        assert http(web, "POST", "/api/runs", defaults)[0] == 409
    finally:
        release.set()
        thread.join()
    assert responses[0][0] == 201


def test_traversal_and_symlink_escape_do_not_read_outside_root(web, tmp_path, http):
    outside = tmp_path.parent / f"{tmp_path.name}-outside"
    outside.mkdir()
    (outside / "record.json").write_text("{}")
    (outside / "metrics.json").write_text("{}")
    (web.directory / "escape").symlink_to(outside, target_is_directory=True)
    assert http(web, "GET", "/api/runs/escape")[0] == 400
    assert http(web, "GET", "/api/runs/../outside")[0] == 400
    assert http(web, "GET", "/api/runs/missing")[0] == 404
    assert json.loads(http(web, "GET", "/api/runs")[1])["ids"] == []


def test_real_unsafe_request_uses_the_highway_env_adapter(web, defaults, http):
    from cutin.interfaces.web.server import run_scenario

    web.runner = run_scenario
    scenario = defaults["scenario"]
    scenario["cutin"].update(x_m=60, speed_mps=20)
    scenario["cutin_controller"].update(
        target_gap_m=3, lane_change_duration_s=1, warmup_s=0
    )
    status, content, _ = http(web, "POST", "/api/runs", defaults)
    assert status == 201
    result = json.loads(content)
    assert result["metrics"]["ego_collision"]
    saved = json.loads(http(web, "GET", f"/api/runs/{result['id']}")[1])
    assert saved["record"]["termination_reason"] == "collision"
    assert saved["record"]["frames"][-1]["tick"] > 0
