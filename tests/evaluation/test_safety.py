"""Synthetic log fixtures define metric semantics without importing a backend."""

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
from cutin.evaluation import EvaluationConfig, evaluate
from cutin.evaluation.safety import observe_pair


def vehicle(role=Role.EGO, x=0, y=4, speed=20, heading=0, collided=False):
    return VehicleState(role, x, y, heading, speed, 5, 2, collided)


def log_for(states, reason=TerminationReason.HORIZON):
    # Each item is a tuple of states at one physics tick.
    return SimulationLog(
        ScenarioConfig(duration_s=max(len(states) - 1, 1) / 30),
        tuple(
            LogEntry(StateSnapshot(tick, actors)) for tick, actors in enumerate(states)
        ),
        reason,
        simulator_settings={"physics_hz": 30},
    )


@pytest.mark.parametrize(
    "x, speed, expected_gap, expected_ttc",
    [
        (15, 15, 10, 2),  # Ego trails and closes.
        (15, 20, 10, None),  # Equal speed.
        (15, 25, 10, None),  # Separating.
        (-15, 25, 10, 2),  # Other trails and closes.
        (-15, 15, 10, None),
        (5, 15, 0, None),  # Touching/overlap never becomes positive TTC.
        (3, 15, -2, None),
    ],
)
def test_signed_gap_and_positive_ttc(x, speed, expected_gap, expected_ttc):
    gap, ttc = observe_pair(vehicle(), vehicle(Role.FRONT, x=x, speed=speed))
    assert gap == expected_gap
    assert ttc == expected_ttc


def test_lateral_applicability_and_partial_intrusion():
    assert observe_pair(vehicle(), vehicle(Role.CUT_IN, x=15, y=0, speed=15)) == (
        None,
        None,
    )
    assert observe_pair(vehicle(), vehicle(Role.CUT_IN, x=15, y=2, speed=15)) == (10, 2)


def test_heading_changes_footprint_and_projected_closing_speed():
    other = vehicle(Role.FRONT, x=15, speed=15, heading=pi / 2)
    gap, ttc = observe_pair(vehicle(), other)
    assert gap == pytest.approx(11.5)
    assert ttc == pytest.approx(11.5 / 20)


def test_minima_use_only_applicable_samples_and_keep_first_tie():
    ego = vehicle()
    original = log_for(
        [
            (ego, vehicle(Role.FRONT, x=25, speed=15)),
            (ego, vehicle(Role.FRONT, x=15, speed=15)),
            (ego, vehicle(Role.FRONT, x=15, speed=15)),
        ]
    )
    result = evaluate(original)
    front, cut_in = result.pairs
    assert front.min_gap_m == 10 and front.min_gap_tick == 1
    assert front.min_ttc_s == 2 and front.min_ttc_tick == 1
    assert cut_in.min_gap_m is cut_in.min_ttc_s is None
    assert result.assessment_status == AssessmentStatus.COMPLETE
    assert result.end_tick == 2 and result.physics_hz == 30
    assert not result.collision


def test_minima_can_occur_at_different_ticks_and_aggregate_same_role():
    ego = vehicle()
    result = evaluate(
        log_for(
            [
                (
                    ego,
                    vehicle(Role.FRONT, x=15, speed=19),
                    vehicle(Role.FRONT, x=30, speed=19),
                ),
                (ego, vehicle(Role.FRONT, x=25, speed=0)),
            ]
        )
    )
    front = result.pairs[0]
    assert front.min_gap_tick == 0 and front.min_gap_m == 10
    assert front.min_ttc_tick == 1 and front.min_ttc_s == 1


def test_negative_gap_is_not_a_collision_flag():
    states = (vehicle(), vehicle(Role.FRONT, x=3, speed=15))
    result = evaluate(log_for([states, states]))
    assert result.pairs[0].min_gap_m == -2
    assert result.gap_warning
    assert not result.collision and not result.ego_collision


@pytest.mark.parametrize(
    "roles, expected_status, ego_collision",
    [
        ((Role.EGO, Role.CUT_IN), AssessmentStatus.COMPLETE, True),
        ((Role.EGO, Role.FRONT), AssessmentStatus.COMPLETE, True),
        ((Role.FRONT, Role.CUT_IN), AssessmentStatus.NON_TARGET_COLLISION, False),
        ((Role.EGO, Role.FRONT, Role.CUT_IN), AssessmentStatus.COMPLETE, True),
    ],
)
def test_collision_evidence_and_assessment(roles, expected_status, ego_collision):
    actors = tuple(
        vehicle(role, x=20 * index, collided=role in roles)
        for index, role in enumerate(Role)
    )
    result = evaluate(log_for([actors], TerminationReason.COLLISION))
    assert result.collision and result.ego_collision == ego_collision
    assert result.first_collision_tick == 0
    assert result.first_ego_collision_tick == (0 if ego_collision else None)
    assert result.assessment_status == expected_status
    assert set(result.collision_roles) == set(roles)


@pytest.mark.parametrize(
    "reason",
    [TerminationReason.ERROR, TerminationReason.CANCELLED, TerminationReason.ROAD_EXIT],
)
def test_partial_assessments_keep_observations_but_are_incomplete(reason):
    actors = (vehicle(collided=True), vehicle(Role.FRONT, x=3, collided=True))
    result = evaluate(log_for([actors], reason))
    assert result.ego_collision
    assert result.assessment_status == AssessmentStatus.INCOMPLETE
    assert result.pairs[0].min_gap_m == -2


def test_empty_error_log_and_missing_applicability_are_null_not_zero():
    result = evaluate(log_for([], TerminationReason.ERROR))
    assert result.assessment_status == AssessmentStatus.INCOMPLETE
    assert result.end_tick is None
    assert all(pair.min_gap_m is pair.min_ttc_s is None for pair in result.pairs)
    encoded = json.dumps(asdict(result), allow_nan=False)
    assert '"min_ttc_s": null' in encoded


def test_thresholds_change_warnings_not_raw_values():
    states = (vehicle(), vehicle(Role.FRONT, x=15, speed=15))
    log = log_for([states, states])
    quiet = evaluate(log, EvaluationConfig(gap_warning_m=10, ttc_warning_s=2))
    warn = evaluate(log, EvaluationConfig(gap_warning_m=11, ttc_warning_s=3))
    assert quiet.pairs == warn.pairs
    assert not quiet.gap_warning and not quiet.ttc_warning
    assert warn.gap_warning and warn.ttc_warning


@pytest.mark.parametrize(
    "config",
    [
        EvaluationConfig(gap_warning_m=-1),
        EvaluationConfig(ttc_warning_s=float("nan")),
        EvaluationConfig(ttc_warning_s=float("inf")),
        EvaluationConfig(gap_warning_m=True),
    ],
)
def test_invalid_thresholds(config):
    with pytest.raises(ValueError):
        evaluate(log_for([], TerminationReason.ERROR), config)


@pytest.mark.parametrize(
    "kind",
    [
        "frequency",
        "ticks",
        "horizon",
        "missing_collision",
        "nan",
        "horizon_collision",
        "after_collision",
    ],
)
def test_malformed_logs_cannot_be_successful_assessments(kind):
    states = (vehicle(), vehicle(Role.FRONT, x=15))
    log = log_for([states, states])
    if kind == "frequency":
        log = replace(log, simulator_settings={})
    elif kind == "ticks":
        log = replace(
            log,
            entries=(
                log.entries[0],
                replace(log.entries[1], snapshot=StateSnapshot(2, states)),
            ),
        )
    elif kind == "horizon":
        log = replace(log, scenario=replace(log.scenario, duration_s=10))
    elif kind == "missing_collision":
        log = replace(log, termination_reason=TerminationReason.COLLISION)
    elif kind == "nan":
        log = log_for(
            [(replace(states[0], x_m=float("nan")), states[1])], TerminationReason.ERROR
        )
    else:
        crashed = (replace(states[0], collided=True), states[1])
        log = log_for(
            [crashed, crashed],
            TerminationReason.HORIZON
            if kind == "horizon_collision"
            else TerminationReason.COLLISION,
        )
    with pytest.raises(ValueError):
        evaluate(log)
