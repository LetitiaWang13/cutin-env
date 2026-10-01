"""Search policy tested with normalized synthetic logs, not backend dynamics."""

from dataclasses import replace

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
from cutin.falsification import SearchConfig, SearchStopReason, search
from cutin.falsification.objective import eligible, rank
from cutin.generation import ParameterRange, SearchSpace


def make_log(
    scenario, reason=TerminationReason.HORIZON, gap=10, closing=5, target=False
):
    def actors(collided):
        return (
            VehicleState(Role.EGO, 50, 4, 0, 20, 5, 2, collided and target),
            VehicleState(Role.FRONT, 110, 4, 0, 25, 5, 2, collided and not target),
            VehicleState(Role.CUT_IN, 55 + gap, 4, 0, 20 - closing, 5, 2, collided),
        )

    snapshots = (
        StateSnapshot(0, actors(False)),
        StateSnapshot(1, actors(reason == TerminationReason.COLLISION)),
    )
    return SimulationLog(
        scenario,
        tuple(LogEntry(s) for s in snapshots),
        reason,
        simulator_settings={"physics_hz": 30},
    )


def base():
    return ScenarioConfig(duration_s=1 / 30)


def test_collision_priority_gap_then_ttc_and_missing_last():
    horizon = evaluate(make_log(base(), gap=1))
    collision = evaluate(
        make_log(base(), TerminationReason.COLLISION, gap=30, target=True)
    )
    assert rank(collision) < rank(horizon)
    assert rank(evaluate(make_log(base(), gap=2))) < rank(
        evaluate(make_log(base(), gap=3))
    )
    assert rank(evaluate(make_log(base(), gap=10, closing=10))) < rank(
        evaluate(make_log(base(), gap=10, closing=5))
    )
    missing = replace(horizon, pairs=())
    assert rank(horizon) < rank(missing)
    assert not eligible(replace(horizon, assessment_status=AssessmentStatus.INCOMPLETE))
    assert not eligible(replace(horizon, ego_collision=True))
    assert not eligible(replace(collision, collision=False))
    with pytest.raises(ValueError):
        rank(evaluate(make_log(base(), TerminationReason.COLLISION)))


def test_budget_stops_and_equal_ranks_keep_first_full_log():
    seen = []

    def simulate(candidate):
        seen.append(candidate)
        return make_log(candidate)

    result = search(base(), SearchSpace(), SearchConfig(3, 10), simulate, evaluate)
    assert result.attempts == result.completed == len(seen) == 3
    assert result.best_attempt == 1 and result.best_config == base()
    assert result.best_log is not None
    assert result.stop_reason == SearchStopReason.BUDGET


def test_rejections_consume_attempt_cap_without_calling_simulator():
    def never(*args):
        pytest.fail("invalid candidate must not reach simulator")

    space = SearchSpace((ParameterRange("initial.front.x_m", 30, 40),))
    result = search(base(), space, SearchConfig(2, 4), never, evaluate)
    assert result.attempts == result.rejected == 4
    assert result.completed == 0 and result.best_config is None
    assert result.stop_reason == SearchStopReason.ATTEMPT_CAP
    assert "front must start ahead" in result.last_rejection


def test_failures_non_target_road_exit_and_incomplete_never_win():
    reasons = iter(
        [
            TerminationReason.ERROR,
            TerminationReason.ROAD_EXIT,
            TerminationReason.COLLISION,
            TerminationReason.HORIZON,
        ]
    )

    def simulate(candidate):
        return make_log(candidate, next(reasons), gap=-2)

    def evaluation(log):
        metrics = evaluate(log)
        return (
            replace(metrics, assessment_status=AssessmentStatus.INCOMPLETE)
            if log.termination_reason == TerminationReason.HORIZON
            else metrics
        )

    result = search(base(), SearchSpace(), SearchConfig(1, 4), simulate, evaluation)
    assert (
        result.errors
        == result.road_exits
        == result.non_target_collisions
        == result.incomplete
        == 1
    )
    assert result.completed == 0 and result.best_log is result.best_metrics is None
    assert result.ego_collisions == 0
    assert result.attempts == sum(
        (
            result.completed,
            result.rejected,
            result.errors,
            result.road_exits,
            result.non_target_collisions,
            result.incomplete,
            result.cancelled,
        )
    )


def test_exceptions_count_and_later_valid_candidate_can_win():
    count = 0

    def simulate(candidate):
        nonlocal count
        count += 1
        if count == 1:
            raise RuntimeError("backend failure")
        return make_log(candidate)

    def evaluation(log):
        if count == 2:
            raise ValueError("bad log")
        return evaluate(log)

    result = search(base(), SearchSpace(), SearchConfig(1, 5), simulate, evaluation)
    assert result.errors == 2 and result.completed == 1 and result.attempts == 3
    assert result.best_attempt == 3 and result.last_error == "ValueError: bad log"


def test_goal_collision_wins_but_search_continues_to_budget():
    calls = 0

    def simulate(candidate):
        nonlocal calls
        calls += 1
        return make_log(
            candidate,
            TerminationReason.COLLISION if calls == 2 else TerminationReason.HORIZON,
            gap=30 if calls == 2 else 1,
            target=True,
        )

    result = search(base(), SearchSpace(), SearchConfig(3, 10), simulate, evaluate)
    assert result.completed == 3 and result.best_attempt == 2
    assert result.ego_collisions == 1
    assert result.best_metrics.ego_collision


def test_cancelled_run_stops_search_and_retains_previous_best():
    calls = 0

    def simulate(candidate):
        nonlocal calls
        calls += 1
        return make_log(
            candidate,
            TerminationReason.HORIZON if calls == 1 else TerminationReason.CANCELLED,
        )

    result = search(base(), SearchSpace(), SearchConfig(5, 10), simulate, evaluate)
    assert result.completed == 1 and result.cancelled == 1 and result.attempts == 2
    assert result.stop_reason == SearchStopReason.CANCELLED and result.best_attempt == 1


def test_cancellation_does_not_require_evaluation_or_launch_another_run():
    def never(*args):
        pytest.fail("cancelled run must not be evaluated")

    result = search(
        base(),
        SearchSpace(),
        SearchConfig(2, 10),
        lambda s: make_log(s, TerminationReason.CANCELLED),
        never,
    )
    assert result.attempts == result.cancelled == 1
    assert result.stop_reason == SearchStopReason.CANCELLED


def test_deterministic_proposals_and_base_seed_unchanged():
    space = SearchSpace((ParameterRange("initial.cut_in.x_m", 45, 80),))
    streams = []
    for _ in range(2):
        seen = []

        def simulate(candidate, seen=seen):
            seen.append(candidate)
            return make_log(candidate)

        search(base(), space, SearchConfig(4, 10, 19), simulate, evaluate)
        streams.append(seen)
    assert streams[0] == streams[1]
    assert len(set(s.initial.cut_in.x_m for s in streams[0])) == 4
    assert all(s.seed == 42 for s in streams[0])


@pytest.mark.parametrize(
    "config",
    [
        SearchConfig(budget=0),
        SearchConfig(max_attempts=-1),
        SearchConfig(seed=True),
        SearchConfig(seed=-1),
    ],
)
def test_invalid_search_configuration_fails_before_callbacks(config):
    def never(*args):
        pytest.fail("invalid settings must not launch a run")

    with pytest.raises(ValueError):
        search(base(), SearchSpace(), config, never, evaluate)


def test_attempt_cap_may_be_lower_than_budget():
    result = search(
        base(), SearchSpace(), SearchConfig(10, 2), lambda s: make_log(s), evaluate
    )
    assert result.completed == 2 and result.stop_reason == SearchStopReason.ATTEMPT_CAP


def test_keyboard_interrupt_is_not_swallowed():
    def interrupt(*args):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        search(base(), SearchSpace(), SearchConfig(), interrupt, evaluate)


def test_mismatched_results_are_errors_not_candidates():
    result = search(
        base(),
        SearchSpace(),
        SearchConfig(1, 2),
        lambda s: make_log(s),
        lambda log: replace(
            evaluate(log), termination_reason=TerminationReason.COLLISION
        ),
    )
    assert result.errors == 2 and result.best_config is None
