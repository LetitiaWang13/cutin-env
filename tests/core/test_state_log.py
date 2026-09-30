"""Internal records carry data; boundary/runner code owns validation."""

from dataclasses import FrozenInstanceError, replace

import pytest

from cutin.core import (
    AppliedAction,
    ControlAction,
    LogEntry,
    Role,
    ScenarioConfig,
    SimulationLog,
    StateSnapshot,
    TerminationReason,
    VehicleState,
)


def vehicle(role: Role = Role.EGO) -> VehicleState:
    return VehicleState(role, 50, 4, 0, 25, 5, 2)


def test_snapshot_does_not_assume_three_vehicles() -> None:
    state = StateSnapshot(tick=60, vehicles={Role.EGO: (vehicle(),)})
    assert state.tick / 30 == 2.0
    assert state.vehicles[Role.EGO][0].speed_mps == 25
    assert StateSnapshot(tick=0, vehicles={}).vehicles == {}
    assert state.vehicles.get(Role.FRONT, ()) == ()


def test_snapshot_supports_multiple_vehicles_per_role() -> None:
    first = vehicle(Role.FRONT)
    second = replace(first, x_m=100)
    state = StateSnapshot(0, {Role.FRONT: (first, second), Role.EGO: ()})
    assert state.vehicles[Role.FRONT] == (first, second)
    assert state.vehicles[Role.EGO] == ()


def test_snapshot_copies_and_freezes_role_mapping() -> None:
    grouped = {Role.EGO: (vehicle(),)}
    state = StateSnapshot(0, grouped)
    grouped.clear()
    assert len(state.vehicles[Role.EGO]) == 1
    with pytest.raises(TypeError):
        state.vehicles[Role.FRONT] = ()
    with pytest.raises(TypeError):
        state.vehicles[Role.EGO][0] = vehicle()
    with pytest.raises(FrozenInstanceError):
        state.tick = 1


def test_records_are_immutable_and_replaceable() -> None:
    state = StateSnapshot(0, {Role.EGO: (vehicle(),)})
    with pytest.raises(FrozenInstanceError):
        state.vehicles[Role.EGO][0].x_m = 0
    action = ControlAction(0, 0)
    assert replace(action, acceleration_mps2=1).acceleration_mps2 == 1
    assert action.acceleration_mps2 == 0


def test_log_keeps_input_actions_and_integer_ticks() -> None:
    initial = LogEntry(StateSnapshot(0, {Role.EGO: (vehicle(),)}))
    terminal = LogEntry(
        StateSnapshot(300, {Role.EGO: (vehicle(),)}),
        (AppliedAction(Role.EGO, ControlAction(0, 0)),),
    )
    scenario = ScenarioConfig()
    log = SimulationLog(scenario, (initial, terminal), TerminationReason.HORIZON)
    assert log.scenario == scenario
    assert log.entries == (initial, terminal)
    assert log.entries[0].applied_actions == ()
    assert log.entries[-1].snapshot.tick == 300
    assert log.entries[-1].applied_actions[0].role == Role.EGO


def test_error_can_preserve_partial_history_and_diagnostic() -> None:
    log = SimulationLog(
        ScenarioConfig(),
        (LogEntry(StateSnapshot(0, {Role.EGO: (vehicle(),)})),),
        TerminationReason.ERROR,
        diagnostic="step failed",
    )
    assert log.termination_reason == TerminationReason.ERROR
    assert log.diagnostic == "step failed"


def test_records_have_no_implicit_validation_hooks() -> None:
    for record in (
        ScenarioConfig,
        VehicleState,
        ControlAction,
        AppliedAction,
        LogEntry,
        SimulationLog,
    ):
        assert not hasattr(record, "__post_init__")
