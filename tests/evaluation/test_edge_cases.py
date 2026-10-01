"""Null semantics and validation at the evaluator input boundary."""

import json
from dataclasses import asdict, replace
from math import pi

import pytest

from cutin.core import (
    AssessmentStatus,
    LogEntry,
    Role,
    ScenarioConfig,
    SimulationLog,
    StateSnapshot,
    TerminationReason,
    VehicleState,
)
from cutin.evaluation import evaluate
from cutin.evaluation.safety import observe_pair


def state(role=Role.EGO, x=0, y=4, speed=20, heading=0):
    return VehicleState(role, x, y, heading, speed, 5, 2)


def partial(actors):
    return SimulationLog(
        ScenarioConfig(),
        (LogEntry(StateSnapshot(0, actors)),),
        TerminationReason.CANCELLED,
        simulator_settings={"physics_hz": 30},
    )


def test_rotated_footprint_can_make_lateral_bounds_applicable():
    assert observe_pair(state(), state(Role.CUT_IN, x=15, y=0.6, speed=15)) == (
        None,
        None,
    )
    gap, ttc = observe_pair(
        state(), state(Role.CUT_IN, x=15, y=0.6, speed=15, heading=pi / 2)
    )
    assert gap == pytest.approx(11.5)
    assert ttc == pytest.approx(11.5 / 20)


def test_all_ego_instances_participate_and_missing_ego_is_unavailable():
    result = evaluate(
        partial((state(), state(x=10), state(Role.FRONT, x=20, speed=15)))
    )
    assert result.pairs[0].min_gap_m == 5
    assert result.pairs[0].min_ttc_s == 1
    no_ego = evaluate(partial((state(Role.FRONT, x=20),)))
    assert all(pair.min_gap_m is pair.min_ttc_s is None for pair in no_ego.pairs)


def test_ttc_minimum_ignores_nonclosing_samples_even_when_gap_is_smaller():
    first = (state(), state(Role.FRONT, x=25, speed=15))
    later = (state(), state(Role.FRONT, x=7, speed=25))
    log = replace(
        partial(first),
        entries=(LogEntry(StateSnapshot(0, first)), LogEntry(StateSnapshot(1, later))),
    )
    pair = evaluate(log).pairs[0]
    assert pair.min_gap_m == 2 and pair.min_gap_tick == 1
    assert pair.min_ttc_s == 4 and pair.min_ttc_tick == 0


@pytest.mark.parametrize("hz", [0, -1, True, 30.0, "30", float("nan")])
def test_frequency_is_integer_clock_metadata(hz):
    with pytest.raises(ValueError, match="physics_hz"):
        evaluate(replace(partial(()), simulator_settings={"physics_hz": hz}))


@pytest.mark.parametrize(
    "field, value",
    [
        ("x_m", float("inf")),
        ("heading_rad", float("nan")),
        ("length_m", 0),
        ("width_m", -1),
        ("speed_mps", -1),
        ("collided", "false"),
        ("role", "ego"),
    ],
)
def test_malformed_states_are_not_silently_treated_as_safe(field, value):
    with pytest.raises(ValueError):
        evaluate(partial((replace(state(), **{field: value}),)))


@pytest.mark.parametrize(
    "reason", [TerminationReason.HORIZON, TerminationReason.COLLISION]
)
def test_missing_target_observations_cannot_establish_complete_assessment(reason):
    actors = (
        replace(state(Role.FRONT), collided=reason == TerminationReason.COLLISION),
    )
    first = (state(), state(Role.FRONT))
    log = replace(
        partial(first),
        scenario=ScenarioConfig(duration_s=1 / 30),
        entries=(LogEntry(StateSnapshot(0, first)), LogEntry(StateSnapshot(1, actors))),
        termination_reason=reason,
    )
    assert evaluate(log).assessment_status == AssessmentStatus.INCOMPLETE


def test_unrepresentable_ttc_is_null_and_serializable():
    result = evaluate(partial((state(speed=1e-310), state(Role.FRONT, x=15, speed=0))))
    assert result.pairs[0].min_gap_m == 10
    assert result.pairs[0].min_ttc_s is None
    json.dumps(asdict(result), allow_nan=False)
    # Overflow of closing speed must not invent a zero ("positive") TTC.
    assert observe_pair(
        state(speed=1e308), state(Role.FRONT, x=15, speed=1e308, heading=pi)
    ) == (10, None)
