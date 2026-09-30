"""Normalized records can be exercised without a simulator."""

from dataclasses import FrozenInstanceError, replace

import pytest

from cutin.core import (
    AppliedAction,
    ControlAction,
    EventKind,
    LogSample,
    Role,
    ScenarioConfig,
    SimulationEvent,
    SimulationLog,
    Snapshot,
    Termination,
    VehicleState,
)


def snapshot(time_s: float = 0, *, collided: bool = False) -> Snapshot:
    return Snapshot(
        time_s=time_s,
        vehicles=tuple(
            VehicleState(
                role=role,
                x_m=50 + 30 * index + 25 * time_s,
                y_m=0 if role == Role.CUT_IN else 4,
                heading_rad=0,
                speed_mps=25,
                length_m=5,
                width_m=2,
                collided=collided and role == Role.EGO,
            )
            for index, role in enumerate(Role)
        ),
    )


def sample(time_s: float, *, collided: bool = False) -> LogSample:
    return LogSample(
        snapshot(time_s, collided=collided),
        tuple(AppliedAction(role, ControlAction(0, 0)) for role in Role)
        if time_s > 0
        else (),
    )


def test_role_identity_does_not_depend_on_order() -> None:
    state = snapshot()
    reversed_state = replace(state, vehicles=tuple(reversed(state.vehicles)))
    assert reversed_state.vehicle(Role.EGO) == state.vehicle(Role.EGO)
    with pytest.raises(ValueError, match="Role"):
        state.vehicle("ego")


def test_immutable_records() -> None:
    with pytest.raises(FrozenInstanceError):
        snapshot().vehicles[0].x_m = 0
    with pytest.raises(FrozenInstanceError):
        ControlAction(0, 0).steering_rad = 1


@pytest.mark.parametrize("value", [True, "1", float("nan"), float("inf")])
def test_invalid_action_values(value: object) -> None:
    with pytest.raises(ValueError):
        ControlAction(value, 0)
    with pytest.raises(ValueError):
        ControlAction(0, value)


@pytest.mark.parametrize(
    "changes",
    [
        {"role": "ego"},
        {"speed_mps": -1},
        {"length_m": 0},
        {"width_m": float("nan")},
        {"collided": 1},
        {"x_m": float("inf")},
    ],
)
def test_invalid_vehicle_state(changes: dict) -> None:
    with pytest.raises(ValueError):
        replace(snapshot().vehicles[0], **changes)


def test_snapshot_requires_all_roles_and_immutable_container() -> None:
    state = snapshot()
    for vehicles in (
        state.vehicles[:2],
        (state.vehicles[0],) * 3,
        list(state.vehicles),
    ):
        with pytest.raises(ValueError):
            Snapshot(0, vehicles)
    with pytest.raises(ValueError):
        replace(state, time_s=-1)


def test_applied_actions_are_keyed_and_complete() -> None:
    with pytest.raises(ValueError, match="initial sample"):
        LogSample(snapshot(), sample(1).applied_actions)
    with pytest.raises(ValueError, match="one applied action"):
        LogSample(snapshot(1))
    with pytest.raises(ValueError, match="one applied action"):
        LogSample(snapshot(1), (AppliedAction(Role.EGO, ControlAction(0, 0)),) * 3)
    with pytest.raises(ValueError, match="tuple"):
        LogSample(snapshot(1), list(sample(1).applied_actions))


def test_horizon_log_keeps_resolved_scenario_and_terminal_sample() -> None:
    log = SimulationLog(ScenarioConfig(), (sample(0), sample(10)), Termination.HORIZON)
    assert log.end_time_s == 10
    assert log.scenario.seed == 42
    with pytest.raises(FrozenInstanceError):
        log.termination = Termination.CANCELLED


def test_collision_and_partial_error_logs_are_distinct() -> None:
    collision = SimulationLog(
        ScenarioConfig(),
        (sample(0), sample(1, collided=True)),
        Termination.COLLISION,
        events=(SimulationEvent(1, EventKind.COLLISION, (Role.EGO,)),),
    )
    assert collision.end_time_s == 1
    error = SimulationLog(
        ScenarioConfig(), (sample(0),), Termination.ERROR, diagnostic="step failed"
    )
    assert error.end_time_s == 0
    assert error.termination != collision.termination


@pytest.mark.parametrize(
    "changes",
    [
        {"samples": ()},
        {"samples": [sample(0)]},
        {"samples": (sample(1),)},
        {"samples": (sample(0), sample(0))},
        {"samples": (sample(0), sample(2), sample(1))},
        {"samples": (sample(0), sample(11))},
        {"termination": "cancelled"},
        {"termination": Termination.HORIZON},
        {"termination": Termination.COLLISION},
        {"termination": Termination.ERROR},
        {"schema_version": True},
        {"events": (SimulationEvent(2, EventKind.MERGE_START, (Role.CUT_IN,)),)},
        {"diagnostic": 1},
    ],
)
def test_invalid_logs(changes: dict) -> None:
    log = SimulationLog(ScenarioConfig(), (sample(0), sample(1)), Termination.CANCELLED)
    with pytest.raises(ValueError):
        replace(log, **changes)


def test_event_validation() -> None:
    for roles in ((), [Role.EGO], (Role.EGO, Role.EGO), ("ego",)):
        with pytest.raises(ValueError):
            SimulationEvent(0, EventKind.COLLISION, roles)
    with pytest.raises(ValueError):
        SimulationEvent(-1, EventKind.COLLISION, (Role.EGO,))
    with pytest.raises(ValueError):
        SimulationEvent(0, "collision", (Role.EGO,))
