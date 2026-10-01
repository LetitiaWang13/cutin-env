"""Real search regressions and packaged CLI/artifact integration."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

from cutin.application import load_scenario, load_search_space, search_scenarios
from cutin.falsification import SearchConfig
from cutin.generation import ParameterRange, SearchSpace

ROOT = Path(__file__).resolve().parents[2]


def test_real_seeded_search_finds_ego_collision_and_repeats_exactly():
    base = load_scenario(ROOT / "examples/baseline.toml")
    space = load_search_space(ROOT / "examples/search_space.toml")
    first = search_scenarios(base, space, SearchConfig(5, 10, 17))
    second = search_scenarios(base, space, SearchConfig(5, 10, 17))
    assert first == second
    assert first.search.completed == first.search.attempts == 5
    assert first.search.best_metrics.ego_collision
    assert first.search.best_attempt == 3
    assert first.search.best_config.seed == base.seed == 42


def test_real_front_only_collision_never_becomes_best():
    base = load_scenario(ROOT / "examples/unsafe.toml")
    space = SearchSpace((ParameterRange("initial.cut_in.x_m", 110, 110),))
    result = search_scenarios(base, space, SearchConfig(1, 2)).search
    assert result.non_target_collisions == result.attempts == 2
    assert result.completed == 0 and result.best_config is None


def test_packaged_search_cli_saves_summary_and_reusable_best_scenario(tmp_path):
    command = shutil.which("cutin")
    assert command
    output = tmp_path / "search"
    args = [
        command,
        "search",
        str(ROOT / "examples/baseline.toml"),
        "--space",
        str(ROOT / "examples/search_space.toml"),
        "--budget",
        "5",
        "--max-attempts",
        "10",
        "--seed",
        "17",
        "--output",
        str(output),
    ]
    completed = subprocess.run(
        args, cwd=tmp_path, check=True, capture_output=True, text=True
    )
    assert "Best ego collision: True" in completed.stdout
    summary = json.loads((output / "search.json").read_text())
    assert summary["best_metrics"]["ego_collision"]
    assert summary["config"]["seed"] == 17 and summary["best_config"]["seed"] == 42
    assert summary["best_metrics"] == json.loads(
        (output / "best/metrics.json").read_text()
    )
    repeated = subprocess.run(args, cwd=tmp_path, capture_output=True)
    assert repeated.returncode == 2
    subprocess.run(
        [command, "run", str(output / "best/scenario.json")],
        cwd=tmp_path,
        check=True,
        capture_output=True,
    )


def test_cli_no_eligible_candidates_are_null_and_budget_is_bounded(tmp_path):
    space = tmp_path / "impossible.json"
    space.write_text(json.dumps({"ranges": {"initial.front.x_m": [30, 40]}}))
    output = tmp_path / "search"
    command = shutil.which("cutin")
    subprocess.run(
        [
            command,
            "search",
            str(ROOT / "examples/baseline.toml"),
            "--space",
            str(space),
            "--budget",
            "2",
            "--max-attempts",
            "3",
            "--output",
            str(output),
        ],
        cwd=tmp_path,
        check=True,
        capture_output=True,
    )
    summary = json.loads((output / "search.json").read_text())
    assert summary["attempts"] == summary["rejected"] == 3
    assert summary["completed"] == 0 and summary["best_config"] is None
    assert summary["stop_reason"] == "attempt_cap"
    assert not (output / "best").exists()


def test_all_backend_errors_save_summary_then_exit_one(tmp_path):
    base = load_scenario(ROOT / "examples/baseline.toml").to_dict()
    base["vehicles"]["width_m"] = 3  # Domain-valid, unsupported by this backend.
    scenario = tmp_path / "unsupported.json"
    scenario.write_text(json.dumps(base))
    space = tmp_path / "fixed.json"
    space.write_text('{"ranges": {}}')
    output = tmp_path / "failed"
    command = shutil.which("cutin")
    completed = subprocess.run(
        [
            command,
            "search",
            str(scenario),
            "--space",
            str(space),
            "--budget",
            "1",
            "--max-attempts",
            "2",
            "--output",
            str(output),
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 1
    summary = json.loads((output / "search.json").read_text())
    assert summary["errors"] == summary["attempts"] == 2
    assert summary["best_config"] is None
    assert "5 m x 2 m" in summary["last_error"]


def test_falsification_import_is_backend_and_application_free():
    code = """
import sys
from cutin.falsification import search
assert not {'numpy', 'highway_env', 'pygame', 'gymnasium'}.intersection(sys.modules)
layers = ('cutin.simulation', 'cutin.evaluation', 'cutin.application')
assert not any(name.startswith(layers) for name in sys.modules)
"""
    subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT)
