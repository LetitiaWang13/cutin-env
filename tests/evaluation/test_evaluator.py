"""Evaluate synthetic motion records with known timings and geometry."""

from dataclasses import replace

import pytest

from cutin.config import EvaluationConfig, SimulationConfig
from cutin.core.record import Frame, ScenarioRecord, VehicleState
from cutin.evaluation.evaluator import Evaluator
from cutin.evaluation.metrics.ttc import PairMinima
from cutin.interfaces.web.inputs import default_scenario


@pytest.fixture
def record() -> ScenarioRecord:
    ego = VehicleState(0, 4, 0, 25, 5, 2, False, None, None)
    initial = Frame(
        tick=0,
        ego=ego,
        front=replace(ego, x_m=20, speed_mps=20),
        cutin=replace(ego, x_m=-15, y_m=0, speed_mps=30),
        cutin_phase="warmup",
    )
    merging = replace(
        initial,
        tick=1,
        front=replace(initial.front, x_m=15),
        cutin=replace(initial.cutin, y_m=3),
        cutin_phase="merge",
    )
    return ScenarioRecord(
        scenario=default_scenario(),
        simulation=SimulationConfig("highway-env", physics_hz=1, controller_hz=1),
        backend_version="test",
        controller_version="test",
        frames=[initial, merging, replace(merging, tick=2)],
        termination_reason="horizon",
        prepare_start_time_s=0.0,
        merge_start_time_s=1.0,
        merge_completion_time_s=None,
    )


def test_evaluation_reports_pair_minima_and_first_ticks(record):
    result = Evaluator(EvaluationConfig()).evaluate(record)

    assert result.pairs == (
        PairMinima("front", 10.0, 1, 2.0, 1),
        PairMinima("cutin", 10.0, 1, 2.0, 1),
    )
    assert result.lane_change_time_s is None
    assert not result.collision
    assert not result.ego_collision
    assert result.collision_roles == ()


def test_ttc_is_unavailable_for_separating_or_laterally_disjoint_pairs(record):
    initial = record.frames[0]
    frame = replace(initial, front=replace(initial.front, speed_mps=30))
    result = Evaluator(EvaluationConfig()).evaluate(replace(record, frames=[frame]))

    assert result.pairs == (
        PairMinima("front", 15.0, 0, None, None),
        PairMinima("cutin", None, None, None, None),
    )


def test_lane_change_time_uses_observed_events(record):
    follow = replace(record.frames[-1], cutin_phase="follow")
    completed = replace(
        record,
        frames=[*record.frames[:-1], follow],
        merge_completion_time_s=2.0,
    )

    assert completed.scenario.cutin_controller.lane_change_duration_s == 2.0
    assert Evaluator(EvaluationConfig()).evaluate(completed).lane_change_time_s == 1.0


@pytest.mark.parametrize(
    ("ego_collided", "roles"),
    [(True, ("ego", "cutin")), (False, ("front", "cutin"))],
)
def test_collision_at_tick_zero_identifies_involved_roles(record, ego_collided, roles):
    initial = record.frames[0]
    frame = replace(
        initial,
        ego=replace(initial.ego, collided=ego_collided),
        front=replace(initial.front, collided=not ego_collided),
        cutin=replace(initial.cutin, collided=True),
    )
    collided = replace(record, frames=[frame], termination_reason="collision")

    result = Evaluator(EvaluationConfig()).evaluate(collided)

    assert result.collision
    assert result.ego_collision is ego_collided
    assert result.first_collision_tick == 0
    assert result.first_ego_collision_tick == (0 if ego_collided else None)
    assert result.collision_roles == roles
