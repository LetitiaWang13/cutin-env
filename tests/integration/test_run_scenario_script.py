"""The normal scenario script executes policies and exports their report."""

import json
import subprocess
import sys
from pathlib import Path


def test_script_saves_controlled_log_and_refuses_overwrite(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[2]
    output = tmp_path / "scenario.json"
    command = [sys.executable, "scripts/run_scenario.py", "--output", str(output)]
    result = subprocess.run(
        command, cwd=root, check=True, capture_output=True, text=True
    )
    assert "Termination: horizon" in result.stdout
    data = json.loads(output.read_text())
    assert data["simulator_settings"]["control_mode"] == "scenario-policy"
    assert data["simulator_settings"]["policy_version"] == "baseline-v1"
    assert data["cut_in_report"]["start_tick"] == 60
    assert data["cut_in_report"]["completion_tick"] is not None
    original = output.read_bytes()
    repeated = subprocess.run(command, cwd=root, capture_output=True)
    assert repeated.returncode != 0
    assert output.read_bytes() == original
