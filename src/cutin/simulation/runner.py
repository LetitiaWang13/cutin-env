"""One execution loop for controlled scenarios or explicit fixed-action probes."""

from collections.abc import Callable, Mapping
from math import isclose, isfinite
from typing import Protocol

from cutin.core import (
    ControlAction,
    LogEntry,
    Role,
    ScenarioConfig,
    SimulationLog,
    StateSnapshot,
    TerminationReason,
    validate_scenario,
)
from cutin.simulation.config import SimulatorConfig, validate_simulator_config
from cutin.simulation.controllers import DrivingPolicy


class SimulationAdapter(Protocol):
    """Small structural seam; implementations need not inherit from this type."""

    def snapshot(self) -> StateSnapshot: ...
    def step(self, actions: Mapping[Role, ControlAction], dt_s: float) -> LogEntry: ...
    def road_exited(self, snapshot: StateSnapshot) -> bool: ...
    def backend_settings(self) -> Mapping[str, str | int | float]: ...
    def close(self) -> None: ...


def _bound_actions(
    actions: Mapping[Role, ControlAction], config: SimulatorConfig
) -> dict[Role, ControlAction]:
    bounded = {}
    for role, action in actions.items():
        if not isfinite(action.acceleration_mps2) or not isfinite(action.steering_rad):
            raise ValueError("actions must be finite")
        bounded[role] = ControlAction(
            min(
                max(action.acceleration_mps2, -config.max_braking_mps2),
                config.max_acceleration_mps2,
            ),
            min(
                max(action.steering_rad, -config.max_steering_rad),
                config.max_steering_rad,
            ),
        )
    return bounded


def simulate(
    scenario: ScenarioConfig,
    config: SimulatorConfig,
    *,
    actions: Mapping[Role, ControlAction] | None = None,
    max_steps: int | None = None,
    adapter_factory: Callable[[ScenarioConfig], SimulationAdapter] | None = None,
) -> SimulationLog:
    """Run scenario policies by default; explicit actions select raw probe mode.

    Generic validation, dt, actuator bounds, stop policy and log assembly live
    here. The adapter counts actual physics ticks; no second clock is maintained.
    A step failure preserves partial history as an error, never a safe outcome.
    """
    validate_scenario(scenario)
    validate_simulator_config(config)
    ticks = scenario.duration_s * config.physics_hz
    if (
        not isfinite(ticks)
        or ticks < 1
        or not isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8)
    ):
        raise ValueError("duration_s must span an integral number of physics ticks")
    if max_steps is not None and (type(max_steps) is not int or max_steps <= 0):
        raise ValueError("max_steps must be a positive integer")
    fixed_actions = _bound_actions(actions, config) if actions is not None else None
    if adapter_factory is None:
        from cutin.simulation.adapters.highway_env import HighwayEnvAdapter

        adapter_factory = HighwayEnvAdapter
    adapter = adapter_factory(scenario)
    entries = []
    settings = config.to_dict()
    reason = TerminationReason.CANCELLED
    diagnostic = "step limit reached (incomplete scenario assessment)"
    horizon_ticks = round(ticks)
    control_interval = config.physics_hz // config.controller_hz
    policy = DrivingPolicy(scenario, config) if actions is None else None
    settings["control_mode"] = (
        "scenario-policy" if policy is not None else "fixed-actions"
    )
    if policy is not None:
        settings["policy_version"] = "baseline-v1"
    try:
        settings.update(adapter.backend_settings())
        initial = adapter.snapshot()
        entries.append(LogEntry(initial))
        commands = fixed_actions if fixed_actions is not None else {}
        for _ in range(min(max_steps or horizon_ticks, horizon_ticks)):
            snapshot = entries[-1].snapshot
            if policy is not None and snapshot.tick % control_interval == 0:
                # Every vehicle decision is computed from this same snapshot.
                commands = _bound_actions(policy.act(snapshot), config)
            entry = adapter.step(commands, 1 / config.physics_hz)
            entries.append(entry)
            if policy is not None:
                policy.observe(entry.snapshot)
            if any(state.collided for state in entry.snapshot.vehicles):
                reason, diagnostic = TerminationReason.COLLISION, None
                break
            if adapter.road_exited(entry.snapshot):
                reason, diagnostic = TerminationReason.ROAD_EXIT, None
                break
            if entry.snapshot.tick == horizon_ticks:
                reason, diagnostic = TerminationReason.HORIZON, None
                break
    except Exception as error:
        reason = TerminationReason.ERROR
        diagnostic = f"{type(error).__name__}: {error}"
    finally:
        adapter.close()
    return SimulationLog(
        scenario,
        tuple(entries),
        reason,
        diagnostic=diagnostic,
        simulator_settings=settings,
        cut_in_report=policy.report() if policy is not None else None,
    )
