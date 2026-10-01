"""Application binds settings; search persistence stores only the winning trace."""

import json
from pathlib import Path

import pytest

from cutin.application import load_search_space, save_search, search_scenarios
from cutin.core import (
    LogEntry,
    Role,
    ScenarioConfig,
    SimulationLog,
    StateSnapshot,
    TerminationReason,
    VehicleState,
)
from cutin.evaluation import EvaluationConfig
from cutin.falsification import SearchConfig
from cutin.generation import SearchSpace
from cutin.simulation import SimulatorConfig


def fake_horizon(scenario, settings):
    actors = (
        VehicleState(Role.EGO, 50, 4, 0, 25, 5, 2),
        VehicleState(Role.FRONT, 110, 4, 0, 25, 5, 2),
    )
    entries = tuple(LogEntry(StateSnapshot(tick, actors)) for tick in (0, 1))
    return SimulationLog(
        scenario,
        entries,
        TerminationReason.HORIZON,
        simulator_settings=settings.to_dict(),
    )


def test_fixed_settings_passed_to_callbacks_and_best_artifacts_agree(tmp_path):
    simulator = SimulatorConfig(physics_hz=20)
    evaluation = EvaluationConfig(gap_warning_m=4)
    seen = []

    def simulate(candidate, config):
        assert config is simulator
        seen.append(candidate)
        return fake_horizon(candidate, config)

    result = search_scenarios(
        ScenarioConfig(duration_s=1 / 20),
        SearchSpace(),
        SearchConfig(2, 4),
        simulator,
        evaluation,
        simulate_fn=simulate,
    )
    directory = save_search(result, tmp_path / "search")
    summary = json.loads((directory / "search.json").read_text())
    assert summary["attempts"] == summary["completed"] == len(seen) == 2
    assert summary["best_attempt"] == 1 and summary["best_run"] == "best"
    assert summary["best_config"] == json.loads(
        (directory / "best/scenario.json").read_text()
    )
    assert summary["best_metrics"] == json.loads(
        (directory / "best/metrics.json").read_text()
    )
    assert summary["best_metrics"]["gap_warning_m"] == 4
    assert summary["best_metrics"]["physics_hz"] == 20
    assert {path.name for path in directory.iterdir()} == {
        "base.json",
        "space.json",
        "search.json",
        "simulator.json",
        "evaluation.json",
        "best",
    }
    assert len(list((directory / "best").iterdir())) == 5
    assert load_search_space(directory / "space.json") == SearchSpace()
    with pytest.raises(FileExistsError):
        save_search(result, directory)


def test_no_eligible_result_is_explicit_null_without_best_directory(tmp_path):
    def failed(scenario, config):
        return SimulationLog(
            scenario,
            (),
            TerminationReason.ERROR,
            diagnostic="backend unavailable",
            simulator_settings=config.to_dict(),
        )

    result = search_scenarios(
        ScenarioConfig(), SearchSpace(), SearchConfig(1, 3), simulate_fn=failed
    )
    directory = save_search(result, tmp_path / "none")
    summary = json.loads((directory / "search.json").read_text())
    assert (
        summary["best_config"] is summary["best_metrics"] is summary["best_run"] is None
    )
    assert summary["errors"] == summary["attempts"] == 3
    assert summary["last_error"] == "backend unavailable"
    assert not (directory / "best").exists()


def test_nested_write_failure_removes_new_search_directory(tmp_path, monkeypatch):
    result = search_scenarios(
        ScenarioConfig(duration_s=1 / 30),
        SearchSpace(),
        SearchConfig(1, 1),
        simulate_fn=fake_horizon,
    )
    original = Path.write_text

    def fail(self, *args, **kwargs):
        if self.name == "log.json":
            raise OSError("disk failure")
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail)
    with pytest.raises(OSError):
        save_search(result, tmp_path / "search")
    assert not (tmp_path / "search").exists()
