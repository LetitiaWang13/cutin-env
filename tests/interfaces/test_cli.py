"""Thin CLI tests with injected workflows, plus actual installed-entry tests."""

import json
import shutil
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest

from cutin.application import run_scenario
from cutin.core import ScenarioConfig, SimulationLog, TerminationReason
from cutin.interfaces import cli

ROOT = Path(__file__).resolve().parents[2]


def error_result():
    def failed(scenario, config):
        return SimulationLog(
            scenario,
            (),
            TerminationReason.ERROR,
            diagnostic="test backend failure",
            simulator_settings=config.to_dict(),
        )

    return run_scenario(ScenarioConfig(), simulate_fn=failed)


def test_cli_saves_failed_run_then_reports_nonzero_status(
    tmp_path, monkeypatch, capsys
):
    monkeypatch.setattr(cli, "run_scenario", lambda *args: error_result())
    directory = tmp_path / "error"
    with pytest.raises(SystemExit) as exit:
        cli.main(
            ["run", str(ROOT / "examples/baseline.toml"), "--output", str(directory)]
        )
    assert exit.value.code == 1
    assert (
        json.loads((directory / "metrics.json").read_text())["assessment_status"]
        == "incomplete"
    )
    assert "test backend failure" in capsys.readouterr().out


def test_cli_refuses_existing_output_before_running(tmp_path, monkeypatch):
    directory = tmp_path / "existing"
    directory.mkdir()

    def never_called(*args):
        pytest.fail("existing destination should fail before physics")

    monkeypatch.setattr(cli, "run_scenario", never_called)
    with pytest.raises(SystemExit) as exit:
        cli.main(
            ["run", str(ROOT / "examples/baseline.toml"), "--output", str(directory)]
        )
    assert exit.value.code == 2


def test_invalid_render_request_fails_before_opening_viewer(monkeypatch):
    def never_called(*args):
        pytest.fail("invalid request must not initialize a viewer")

    monkeypatch.setattr(cli, "run_with_viewer", never_called)
    with pytest.raises(SystemExit) as exit:
        cli.main(
            [
                "run",
                str(ROOT / "examples/baseline.toml"),
                "--render",
                "--ttc-warning-s",
                "-1",
            ]
        )
    assert exit.value.code == 2


def test_cli_native_terminal_close_preserves_horizon_artifacts(tmp_path, monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    import pygame

    from cutin.simulation.adapters.highway_env import HighwayEnvAdapter

    pygame.quit()
    original = HighwayEnvAdapter.render

    def close_terminal(self):
        if self.episode_finished and not self.closed:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return original(self)

    monkeypatch.setattr(HighwayEnvAdapter, "render", close_terminal)
    directory = tmp_path / "visual"
    try:
        cli.main(
            [
                "run",
                str(ROOT / "examples/baseline.toml"),
                "--render",
                "--output",
                str(directory),
            ]
        )
        log = json.loads((directory / "log.json").read_text())
        assert log["termination_reason"] == "horizon"
        assert log["entries"][-1]["snapshot"]["tick"] == 300
        assert not pygame.display.get_init()
    finally:
        pygame.quit()


def test_cli_passes_warning_thresholds_and_can_run_without_writing(monkeypatch, capsys):
    calls = []

    def fake(scenario, simulator, evaluation):
        calls.append(evaluation)
        result = error_result()
        return replace(
            result,
            log=replace(result.log, termination_reason=TerminationReason.CANCELLED),
        )

    monkeypatch.setattr(cli, "run_scenario", fake)
    cli.main(
        [
            "run",
            str(ROOT / "examples/baseline.toml"),
            "--gap-warning-m",
            "4",
            "--ttc-warning-s",
            "3",
        ]
    )
    assert calls[0].gap_warning_m == 4 and calls[0].ttc_warning_s == 3
    assert "Saved:" not in capsys.readouterr().out


@pytest.mark.parametrize(
    "args",
    [
        [],
        ["run"],
        ["run", "missing.toml"],
        ["run", str(ROOT / "examples/baseline.toml"), "--ttc-warning-s", "-1"],
    ],
)
def test_usage_input_and_config_errors_return_two(args):
    with pytest.raises(SystemExit) as exit:
        cli.main(args)
    assert exit.value.code == 2


@pytest.mark.parametrize("name, collision", [("baseline", False), ("unsafe", True)])
def test_packaged_command_runs_outside_repository_and_reuses_saved_json(
    tmp_path, name, collision
):
    command = shutil.which("cutin")
    assert command is not None
    directory = tmp_path / "artifacts"
    completed = subprocess.run(
        [
            command,
            "run",
            str(ROOT / f"examples/{name}.toml"),
            "--output",
            str(directory),
        ],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )
    metrics = json.loads((directory / "metrics.json").read_text())
    assert metrics["ego_collision"] is collision
    assert metrics["assessment_status"] == "complete"
    assert "Assessment: complete" in completed.stdout
    rerun = subprocess.run(
        [command, "run", str(directory / "scenario.json")],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )
    assert f"Collision / ego collision: {collision} {collision}" in rerun.stdout
