"""Execution policy is testable without a highway-env object or vehicle physics."""

from dataclasses import replace

import pytest

from cutin.core import (
    ControlAction,
    LogEntry,
    Role,
    ScenarioConfig,
    StateSnapshot,
    TerminationReason,
    VehicleState,
)
from cutin.simulation import SimulatorConfig, simulate


class FakeAdapter:
    def __init__(self, *, fail_at=None, collision_at=None, exit_at=None):
        self.tick = 0
        self.calls = []
        self.closed = False
        self.fail_at = fail_at
        self.collision_at = collision_at
        self.exit_at = exit_at

    def snapshot(self):
        return StateSnapshot(
            self.tick,
            (
                VehicleState(
                    Role.EGO,
                    50,
                    4,
                    0,
                    25,
                    5,
                    2,
                    collided=self.tick == self.collision_at,
                ),
            ),
        )

    def step(self, actions, dt_s):
        self.calls.append((dict(actions), dt_s))
        if self.tick + 1 == self.fail_at:
            raise RuntimeError("test backend failure")
        self.tick += 1
        return LogEntry(self.snapshot())

    def road_exited(self, snapshot):
        return snapshot.tick == self.exit_at

    def backend_settings(self):
        return {"backend": "test-backend"}

    def close(self):
        self.closed = True


def run_with(adapter, **kwargs):
    return simulate(
        ScenarioConfig(duration_s=0.2),
        SimulatorConfig(physics_hz=20),
        adapter_factory=lambda scenario: adapter,
        **kwargs,
    )


def test_horizon_dt_log_and_cleanup() -> None:
    adapter = FakeAdapter()
    log = run_with(adapter)
    assert log.termination_reason == TerminationReason.HORIZON
    assert [entry.snapshot.tick for entry in log.entries] == [0, 1, 2, 3, 4]
    assert all(dt == 0.05 for _, dt in adapter.calls)
    assert all(
        actions == {Role.EGO: ControlAction(0, 0)} for actions, _ in adapter.calls
    )
    assert log.simulator_settings["physics_hz"] == 20
    assert log.simulator_settings["backend"] == "test-backend"
    assert adapter.closed


def test_generic_action_bounds_belong_to_runner() -> None:
    adapter = FakeAdapter()
    run_with(adapter, actions={Role.EGO: ControlAction(100, 10)})
    assert adapter.calls[0][0] == {Role.EGO: ControlAction(3, 0.5)}
    adapter = FakeAdapter()
    run_with(adapter, actions={Role.EGO: ControlAction(-100, -10)})
    assert adapter.calls[0][0] == {Role.EGO: ControlAction(-6, -0.5)}


def test_step_limit_is_incomplete_not_horizon() -> None:
    adapter = FakeAdapter()
    log = run_with(adapter, max_steps=2)
    assert log.termination_reason == TerminationReason.CANCELLED
    assert log.entries[-1].snapshot.tick == 2
    assert log.diagnostic
    assert adapter.closed


@pytest.mark.parametrize(
    "mode, reason",
    [
        ({"collision_at": 2}, TerminationReason.COLLISION),
        ({"exit_at": 2}, TerminationReason.ROAD_EXIT),
    ],
)
def test_terminal_conditions(mode, reason) -> None:
    adapter = FakeAdapter(**mode)
    log = run_with(adapter)
    assert log.termination_reason == reason
    assert len(adapter.calls) == 2
    assert adapter.closed


def test_errors_preserve_partial_history_and_close() -> None:
    adapter = FakeAdapter(fail_at=2)
    log = run_with(adapter)
    assert log.termination_reason == TerminationReason.ERROR
    assert [entry.snapshot.tick for entry in log.entries] == [0, 1]
    assert "test backend failure" in log.diagnostic
    assert adapter.closed


@pytest.mark.parametrize("duration", [-1, 0.01, 1e-20])
def test_invalid_scenario_fails_before_backend_creation(duration) -> None:
    def factory(scenario):
        pytest.fail("backend must not be created for invalid input")

    with pytest.raises(ValueError):
        simulate(
            replace(ScenarioConfig(), duration_s=duration),
            SimulatorConfig(),
            adapter_factory=factory,
        )


def test_invalid_action_and_step_limit_are_input_errors() -> None:
    with pytest.raises(ValueError, match="finite"):
        run_with(FakeAdapter(), actions={Role.EGO: ControlAction(float("nan"), 0)})
    with pytest.raises(ValueError, match="max_steps"):
        run_with(FakeAdapter(), max_steps=0)
