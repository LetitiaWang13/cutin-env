"""Real driving traces connect to the evaluator without exposing backend objects."""

import json
import subprocess
import sys
import tomllib
from dataclasses import asdict, replace
from pathlib import Path

import pytest

from cutin.core import AssessmentStatus, Role, ScenarioConfig
from cutin.evaluation import evaluate
from cutin.simulation import SimulatorConfig, simulate

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("name, collision", [("baseline", False), ("unsafe", True)])
def test_real_regression_metrics(name, collision):
    scenario = ScenarioConfig.from_dict(
        tomllib.loads((ROOT / "examples" / f"{name}.toml").read_text())
    )
    log = simulate(scenario, SimulatorConfig())
    metrics = evaluate(log)
    assert metrics.assessment_status == AssessmentStatus.COMPLETE
    assert metrics.collision == metrics.ego_collision == collision
    assert metrics.physics_hz == 30
    assert metrics.pairs[0].min_gap_m == pytest.approx(55)
    cut_in = metrics.pairs[1]
    if collision:
        assert metrics.first_collision_tick == metrics.first_ego_collision_tick == 18
        assert set(metrics.collision_roles) == {Role.EGO, Role.CUT_IN}
        assert cut_in.min_gap_m < 0 and metrics.gap_warning
    else:
        assert metrics.end_tick == 300
        assert cut_in.min_gap_m == pytest.approx(12.7974086964)
        assert not metrics.gap_warning
    # Both traces have no positive longitudinal TTC: unsafe merge intrudes
    # sideways into an already-overlapping x footprint, not a closing leader.
    assert all(pair.min_ttc_s is None for pair in metrics.pairs)
    json.dumps(asdict(metrics), allow_nan=False)
    assert evaluate(log) == metrics


def test_real_non_target_collision_is_excluded_from_complete_assessment():
    scenario = ScenarioConfig.from_dict(
        tomllib.loads((ROOT / "examples/unsafe.toml").read_text())
    )
    scenario = replace(
        scenario,
        initial=replace(
            scenario.initial,
            cut_in=replace(scenario.initial.cut_in, x_m=scenario.initial.front.x_m),
        ),
    )
    metrics = evaluate(simulate(scenario, SimulatorConfig()))
    assert metrics.collision and not metrics.ego_collision
    assert set(metrics.collision_roles) == {Role.FRONT, Role.CUT_IN}
    assert metrics.assessment_status == AssessmentStatus.NON_TARGET_COLLISION


def test_evaluation_import_does_not_load_backend():
    script = """
import sys
from cutin.evaluation import EvaluationConfig, evaluate
assert not {'highway_env', 'pygame', 'numpy', 'gymnasium'}.intersection(sys.modules)
assert not any(name.startswith('cutin.simulation') for name in sys.modules)
"""
    subprocess.run([sys.executable, "-c", script], check=True, cwd=ROOT)
