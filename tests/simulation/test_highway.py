"""Real backend checks; no controllers, reward, renderer, or optimizer."""

from dataclasses import replace

import pygame
import pytest

from cutin.core import (
    ControlAction,
    InitialConditions,
    InitialVehicle,
    Role,
    ScenarioConfig,
    VehicleDimensions,
)
from cutin.simulation import SimulatorConfig
from cutin.simulation.adapters.highway import HighwayAdapter


def coast() -> dict[Role, ControlAction]:
    return {role: ControlAction(0, 0) for role in Role}


@pytest.fixture
def adapter():
    instance = HighwayAdapter(ScenarioConfig(), SimulatorConfig())
    yield instance
    instance.close()


def test_exact_initial_scene(adapter: HighwayAdapter) -> None:
    state = adapter.snapshot()
    assert state.tick == 0
    assert len(state.vehicles) == 3
    for role, x, y in ((Role.EGO, 50, 4), (Role.FRONT, 110, 4), (Role.CUT_IN, 65, 0)):
        (vehicle,) = state.vehicles_for_role(role)
        assert (vehicle.x_m, vehicle.y_m, vehicle.speed_mps) == (x, y, 25)
        assert (vehicle.length_m, vehicle.width_m) == (5, 2)
        assert not vehicle.collided
    # One backend-specific assertion checks the actual empty third lane exists.
    assert len(adapter._road.network.graph["start"]["end"]) == 3
    assert len(adapter._road.vehicles) == 3


def test_fixed_step_and_previous_snapshot_is_not_mutated(
    adapter: HighwayAdapter,
) -> None:
    initial = adapter.snapshot()
    result = adapter.step(coast())
    (ego,) = result.snapshot.vehicles_for_role(Role.EGO)
    assert result.snapshot.tick == 1
    assert ego.x_m == pytest.approx(50 + 25 / 30)
    assert initial.vehicles_for_role(Role.EGO)[0].x_m == 50
    assert len(result.applied_actions) == 3


def test_external_commands_survive_backend_act(adapter: HighwayAdapter) -> None:
    actions = coast()
    actions[Role.CUT_IN] = ControlAction(2, 0.1)
    result = adapter.step(actions)
    (cut_in,) = result.snapshot.vehicles_for_role(Role.CUT_IN)
    assert cut_in.speed_mps == pytest.approx(25 + 2 / 30)
    assert cut_in.y_m > 0
    assert cut_in.heading_rad > 0
    applied = {entry.role: entry.action for entry in result.applied_actions}
    assert applied[Role.CUT_IN] == actions[Role.CUT_IN]
    assert applied[Role.EGO] == ControlAction(0, 0)


def test_limits_record_actual_bounded_action(adapter: HighwayAdapter) -> None:
    actions = coast()
    actions[Role.CUT_IN] = ControlAction(100, 10)
    result = adapter.step(actions)
    applied = {entry.role: entry.action for entry in result.applied_actions}
    assert applied[Role.CUT_IN] == ControlAction(3, 0.5)


@pytest.mark.parametrize("speed, requested, expected", [(0.01, -100, 0), (40, 100, 40)])
def test_speed_bounds_prevent_reverse_and_overshoot(speed, requested, expected) -> None:
    scenario = replace(
        ScenarioConfig(),
        initial=InitialConditions(ego=InitialVehicle(50, 1, speed)),
    )
    adapter = HighwayAdapter(scenario, SimulatorConfig())
    try:
        actions = coast()
        actions[Role.EGO] = ControlAction(requested, 0)
        result = adapter.step(actions)
        assert result.snapshot.vehicles_for_role(Role.EGO)[
            0
        ].speed_mps == pytest.approx(expected)
    finally:
        adapter.close()


def test_invalid_commands_do_not_partially_advance(adapter: HighwayAdapter) -> None:
    initial = adapter.snapshot()
    with pytest.raises(ValueError, match="each initialized"):
        adapter.step({Role.EGO: ControlAction(0, 0)})
    actions = coast()
    actions[Role.CUT_IN] = ControlAction(float("nan"), 0)
    with pytest.raises(ValueError, match="finite"):
        adapter.step(actions)
    assert adapter.snapshot() == initial
    assert adapter.step(coast()).snapshot.tick == 1


def test_ego_collision_flags_are_normalized() -> None:
    scenario = replace(
        ScenarioConfig(),
        initial=InitialConditions(front=InitialVehicle(60, 1, 0)),
    )
    adapter = HighwayAdapter(scenario, SimulatorConfig())
    try:
        for _ in range(30):
            state = adapter.step(coast()).snapshot
            if any(vehicle.collided for vehicle in state.vehicles):
                break
        assert state.vehicles_for_role(Role.EGO)[0].collided
        assert state.vehicles_for_role(Role.FRONT)[0].collided
        assert not state.vehicles_for_role(Role.CUT_IN)[0].collided
    finally:
        adapter.close()


def test_non_ego_collision_keeps_ego_flag_clear() -> None:
    scenario = replace(
        ScenarioConfig(),
        initial=InitialConditions(
            front=InitialVehicle(110, 1, 25), cut_in=InitialVehicle(110, 0, 25)
        ),
    )
    adapter = HighwayAdapter(scenario, SimulatorConfig())
    try:
        actions = coast()
        actions[Role.CUT_IN] = ControlAction(0, 0.15)
        for _ in range(30):
            state = adapter.step(actions).snapshot
            if any(vehicle.collided for vehicle in state.vehicles):
                break
        assert state.vehicles_for_role(Role.FRONT)[0].collided
        assert state.vehicles_for_role(Role.CUT_IN)[0].collided
        assert not state.vehicles_for_role(Role.EGO)[0].collided
    finally:
        adapter.close()


def test_repeatability_and_run_isolation() -> None:
    def run():
        adapter = HighwayAdapter(ScenarioConfig(), SimulatorConfig())
        try:
            actions = coast()
            actions[Role.CUT_IN] = ControlAction(1, 0.02)
            return tuple(adapter.step(actions) for _ in range(30))
        finally:
            adapter.close()

    assert run() == run()


def test_headless_does_not_initialize_pygame_display(monkeypatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.setenv("SDL_VIDEODRIVER", "invalid-driver")
    assert not pygame.display.get_init()
    adapter = HighwayAdapter(ScenarioConfig(), SimulatorConfig())
    try:
        adapter.step(coast())
        assert not pygame.display.get_init()
    finally:
        adapter.close()


def test_metadata_road_exit_and_close(adapter: HighwayAdapter) -> None:
    settings = adapter.simulator_settings()
    assert settings["backend"] == "highway"
    assert settings["backend_version"]
    assert settings["physics_hz"] == 30
    state = adapter.snapshot()
    assert not adapter.road_exited(state)
    outside = replace(state, vehicles=(replace(state.vehicles[0], y_m=-3),))
    assert adapter.road_exited(outside)
    adapter.close()
    adapter.close()
    with pytest.raises(RuntimeError, match="closed"):
        adapter.step(coast())


@pytest.mark.parametrize(
    "scenario",
    [
        replace(ScenarioConfig(), duration_s=0.01),
        replace(ScenarioConfig(), duration_s=1e-20),
        replace(ScenarioConfig(), duration_s=-1),
        replace(ScenarioConfig(), vehicles=VehicleDimensions(length_m=6)),
        replace(
            ScenarioConfig(), initial=InitialConditions(ego=InitialVehicle(50, 1, 41))
        ),
    ],
)
def test_unsupported_scenarios_fail_at_adapter_boundary(scenario) -> None:
    with pytest.raises(ValueError):
        HighwayAdapter(scenario, SimulatorConfig())
