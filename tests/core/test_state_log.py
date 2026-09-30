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
    Snapshot,
    TerminationReason,
    VehicleState,
)


def vehicle(role: Role = Role.EGO) -> VehicleState:
    return VehicleState(role, 50, 4, 0, 25, 5, 2)


def test_snapshot_does_not_assume_three_vehicles() -> None:
    state = Snapshot(tick=60, vehicles=(vehicle(),))
    assert state.tick / 30 == 2.0
    assert state.vehicle(Role.EGO).speed_mps == 25
    assert Snapshot(tick=0, vehicles=()).vehicles == ()
    with pytest.raises(KeyError):
        state.vehicle(Role.FRONT)


def test_vehicle_identity_is_not_list_order() -> None:
    ego, front = vehicle(), vehicle(Role.FRONT)
    state = Snapshot(0, (front, ego))
    assert state.vehicle(Role.EGO) == ego


def test_records_are_immutable_and_replaceable() -> None:
    state = Snapshot(0, (vehicle(),))
    with pytest.raises(FrozenInstanceError):
        state.vehicles[0].x_m = 0
    action = ControlAction(0, 0)
    assert replace(action, acceleration_mps2=1).acceleration_mps2 == 1
    assert action.acceleration_mps2 == 0


def test_log_keeps_input_actions_and_integer_ticks() -> None:
    initial = LogEntry(Snapshot(0, (vehicle(),)))
    terminal = LogEntry(
        Snapshot(300, (vehicle(),)),
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
        (LogEntry(Snapshot(0, (vehicle(),))),),
        TerminationReason.ERROR,
        diagnostic="step failed",
    )
    assert log.termination_reason == TerminationReason.ERROR
    assert log.diagnostic == "step failed"


def test_records_have_no_implicit_validation_hooks() -> None:
    for record in (
        ScenarioConfig,
        VehicleState,
        Snapshot,
        ControlAction,
        AppliedAction,
        LogEntry,
        SimulationLog,
    ):
        assert not hasattr(record, "__post_init__")
