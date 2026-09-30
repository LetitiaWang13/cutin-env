"""The probe is a runnable physics diagnostic, not a complete scenario runner."""

import json
import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.parametrize("steps, reason", [(3, "cancelled"), (300, "horizon")])
def test_probe_saves_resolved_settings_and_terminal_tick(
    tmp_path, steps, reason
) -> None:
    root = Path(__file__).resolve().parents[2]
    output = tmp_path / "probe.json"
    command = [
        sys.executable,
        "scripts/probe_highway_env.py",
        "--steps",
        str(steps),
        "--acceleration",
        "0",
        "--steering",
        "0",
        "--output",
        str(output),
    ]
    result = subprocess.run(
        command, cwd=root, check=True, capture_output=True, text=True
    )
    assert "scenario cut-in timing/policy is not executed" in result.stdout
    data = json.loads(output.read_text())
    assert data["termination_reason"] == reason
    assert data["simulator_settings"]["physics_hz"] == 30
    assert data["simulator_settings"]["backend"] == "highway-env"
    assert data["simulator_settings"]["control_mode"] == "fixed-actions"
    assert data["cut_in_report"] is None
    assert data["simulator_settings"]["backend_version"]
    assert data["scenario"]["cut_in"]["start_time_s"] == 2
    assert [entry["snapshot"]["tick"] for entry in data["entries"]] == list(
        range(steps + 1)
    )
    assert data["entries"][0]["applied_actions"] == []
    assert len(data["entries"][-1]["applied_actions"]) == 3
    original = output.read_bytes()
    repeated = subprocess.run(command, cwd=root, capture_output=True)
    assert repeated.returncode != 0
    assert output.read_bytes() == original
