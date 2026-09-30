"""Translation is separate from lifecycle/stepping and does not mutate input."""

from cutin.core import ControlAction, Role, ScenarioConfig
from cutin.simulation.adapters.highway_env_translation import (
    build_scene,
    collect_snapshot,
    translate_actions,
)


def test_action_translation_maps_fields_without_generic_actuator_clamping() -> None:
    road, vehicles = build_scene(ScenarioConfig())
    actions = {role: ControlAction(0, 0) for role in Role}
    actions[Role.EGO] = ControlAction(4, 0.1)
    translated = translate_actions(actions, vehicles, 1 / 30)
    assert translated[Role.EGO] == {"acceleration": 4, "steering": 0.1}
    assert actions[Role.EGO] == ControlAction(4, 0.1)
    assert vehicles[Role.EGO].action == {"acceleration": 0, "steering": 0}
    assert len(road.vehicles) == 3


def test_state_translation_copies_backend_values() -> None:
    _, vehicles = build_scene(ScenarioConfig())
    original = collect_snapshot(vehicles, 7)
    vehicles[Role.EGO].position[0] = 100
    assert original.tick == 7
    assert original.vehicles_for_role(Role.EGO)[0].x_m == 50
    assert collect_snapshot(vehicles, 8).vehicles_for_role(Role.EGO)[0].x_m == 100
