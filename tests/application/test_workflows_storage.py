"""Composition and filesystem policy tested without backend objects."""

import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest

from cutin.application import load_scenario, run_scenario, save_run
from cutin.core import (
    LogEntry,
    ScenarioConfig,
    SimulationLog,
    StateSnapshot,
    TerminationReason,
)
from cutin.evaluation import EvaluationConfig, evaluate
from cutin.simulation import SimulatorConfig


def fake_log(scenario, simulator):
    return SimulationLog(
        scenario,
        (LogEntry(StateSnapshot(0, ())),),
        TerminationReason.CANCELLED,
        simulator_settings=simulator.to_dict(),
    )


def result():
    return run_scenario(ScenarioConfig(), simulate_fn=fake_log)


def test_workflow_passes_exact_configs_and_evaluates_returned_log():
    scenario = ScenarioConfig()
    simulator = SimulatorConfig(physics_hz=20)
    evaluation = EvaluationConfig(ttc_warning_s=2)
    calls = []

    def simulate_fn(actual_scenario, actual_simulator):
        assert actual_scenario is scenario and actual_simulator is simulator
        calls.append("simulate")
        return fake_log(actual_scenario, actual_simulator)

    def evaluate_fn(log, actual_evaluation):
        assert actual_evaluation is evaluation
        calls.append("evaluate")
        return evaluate(log, actual_evaluation)

    run = run_scenario(
        scenario,
        simulator,
        evaluation,
        simulate_fn=simulate_fn,
        evaluate_fn=evaluate_fn,
    )
    assert calls == ["simulate", "evaluate"]
    assert run.simulator_config is simulator and run.evaluation_config is evaluation
    assert run.metrics.physics_hz == 20
    assert run.metrics.ttc_warning_s == 2


def test_invalid_evaluation_settings_do_not_spend_a_simulation():
    def never_called(*args):
        pytest.fail("simulation must not start")

    with pytest.raises(ValueError):
        run_scenario(
            ScenarioConfig(),
            evaluation_config=EvaluationConfig(ttc_warning_s=-1),
            simulate_fn=never_called,
        )


def test_save_and_reload_scenario_and_consistent_artifact_views(tmp_path):
    run = result()
    directory = save_run(run, tmp_path / "nested" / "run")
    assert {path.name for path in directory.iterdir()} == {
        "scenario.json",
        "simulator.json",
        "evaluation.json",
        "log.json",
        "metrics.json",
    }
    assert load_scenario(directory / "scenario.json") == run.log.scenario
    documents = {
        path.name: json.loads(path.read_text()) for path in directory.iterdir()
    }
    assert documents["scenario.json"] == documents["log.json"]["scenario"]
    assert (
        documents["simulator.json"]["physics_hz"]
        == documents["metrics.json"]["physics_hz"]
    )
    assert (
        documents["evaluation.json"]["ttc_warning_s"]
        == documents["metrics.json"]["ttc_warning_s"]
    )
    assert documents["metrics.json"]["assessment_status"] == "incomplete"
    assert documents["log.json"]["termination_reason"] == "cancelled"


@pytest.mark.parametrize("kind", ["directory", "file", "symlink"])
def test_existing_outputs_are_never_overwritten(tmp_path, kind):
    directory = tmp_path / "run"
    if kind == "directory":
        directory.mkdir()
        (directory / "sentinel").write_text("keep")
    elif kind == "file":
        directory.write_text("keep")
    else:
        directory.symlink_to(tmp_path / "missing-target")
    with pytest.raises(FileExistsError):
        save_run(result(), directory)
    if kind == "directory":
        assert (directory / "sentinel").read_text() == "keep"
    elif kind == "file":
        assert directory.read_text() == "keep"
    else:
        assert directory.is_symlink()


def test_invalid_json_values_do_not_create_an_output_directory(tmp_path):
    run = result()
    run = replace(run, metrics=replace(run.metrics, gap_warning_m=float("nan")))
    directory = tmp_path / "run"
    with pytest.raises(ValueError):
        save_run(run, directory)
    assert not directory.exists()


def test_write_failure_removes_only_the_new_run_directory(tmp_path, monkeypatch):
    directory = tmp_path / "run"
    sentinel = tmp_path / "keep.txt"
    sentinel.write_text("keep")
    original = Path.write_text

    def fail_log_write(self, *args, **kwargs):
        if self.name == "log.json":
            raise OSError("disk failure")
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail_log_write)
    with pytest.raises(OSError, match="disk failure"):
        save_run(result(), directory)
    assert not directory.exists()
    assert sentinel.read_text() == "keep"


@pytest.mark.parametrize(
    "name, text",
    [
        ("bad.yaml", "x: 1"),
        ("bad.json", "[]"),
        ("bad.json", "{"),
        ("bad.toml", "duration_s = -1"),
    ],
)
def test_invalid_scenario_files(name, text, tmp_path):
    path = tmp_path / name
    path.write_text(text)
    with pytest.raises(ValueError):
        load_scenario(path)


def test_application_import_is_backend_free():
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import cutin.application; "
            "assert not {'highway_env', 'pygame', 'numpy', 'gymnasium'}"
            ".intersection(sys.modules)",
        ],
        check=True,
    )
