"""A sampled concrete scenario uses the existing run workflow and CLI unchanged."""

import json
import subprocess
import sys
import tomllib
from pathlib import Path
from random import Random

from cutin.application import load_scenario, run_scenario
from cutin.core import AssessmentStatus, ScenarioConfig
from cutin.generation import SearchSpace, sample

ROOT = Path(__file__).resolve().parents[2]


def test_example_proposal_is_reproducible_and_runs_through_existing_workflow():
    base = load_scenario(ROOT / "examples/baseline.toml")
    space = SearchSpace.from_dict(
        tomllib.loads((ROOT / "examples/search_space.toml").read_text())
    )
    candidate = sample(base, space, Random(17))
    assert candidate == sample(base, space, Random(17))
    result = run_scenario(candidate)
    assert result.log.scenario == candidate
    assert result.metrics.assessment_status == AssessmentStatus.COMPLETE
    assert candidate.seed == base.seed == 42


def test_sampling_script_exports_valid_scenario_and_never_overwrites(tmp_path):
    output = tmp_path / "candidate.json"
    command = [
        sys.executable,
        "scripts/sample_scenario.py",
        "--seed",
        "17",
        "--output",
        str(output),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    candidate = load_scenario(output)
    assert candidate.seed == 42
    original = output.read_bytes()
    repeated = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    assert repeated.returncode == 2
    assert output.read_bytes() == original
    stdout = subprocess.run(
        [sys.executable, "scripts/sample_scenario.py", "--seed", "17"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert ScenarioConfig.from_dict(json.loads(stdout.stdout)) == candidate


def test_script_rejects_once_and_creates_no_output(tmp_path):
    space_file = tmp_path / "impossible.toml"
    space_file.write_text('[ranges]\n"initial.front.x_m" = [30, 40]\n')
    output = tmp_path / "candidate.json"
    completed = subprocess.run(
        [
            sys.executable,
            "scripts/sample_scenario.py",
            "--space",
            str(space_file),
            "--output",
            str(output),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 1
    assert "Rejected sample:" in completed.stderr
    assert not output.exists()


def test_generation_import_has_no_backend_or_execution_layer_dependency():
    code = """
import sys
from cutin.generation import SearchSpace, sample
assert not {'numpy', 'highway_env', 'pygame', 'gymnasium'}.intersection(sys.modules)
layers = ('cutin.simulation', 'cutin.evaluation', 'cutin.application')
assert not any(name.startswith(layers) for name in sys.modules)
"""
    subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT)
