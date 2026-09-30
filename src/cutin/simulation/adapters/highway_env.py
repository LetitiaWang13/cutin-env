"""Lifecycle/stepping bridge to highway-env; no scheduling or generic config."""

from collections.abc import Mapping
from importlib.metadata import version
from math import isfinite

from cutin.core import (
    AppliedAction,
    ControlAction,
    LogEntry,
    Role,
    ScenarioConfig,
    StateSnapshot,
)
from cutin.simulation.adapters.highway_env_translation import (
    build_scene,
    collect_snapshot,
    road_exited,
    translate_actions,
)


class HighwayEnvAdapter:
    """A validated scenario becomes fresh backend objects; caller supplies dt.

    Translation helpers own backend representation/limitations. The runner owns
    frequencies, policy limits, horizon, and log assembly. No SimulatorConfig
    dependency here, and no stock traffic, rewards, or renderer.
    """

    def __init__(self, scenario: ScenarioConfig) -> None:
        self._geometry = scenario.road
        self._road, self._vehicles = build_scene(scenario)
        self._tick = 0
        self._closed = False

    def snapshot(self) -> StateSnapshot:
        return collect_snapshot(self._vehicles, self._tick)

    def step(self, actions: Mapping[Role, ControlAction], dt_s: float) -> LogEntry:
        if self._closed:
            raise RuntimeError("adapter is closed")
        if not isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        translated = translate_actions(actions, self._vehicles, dt_s)
        for role, vehicle in self._vehicles.items():
            vehicle.act(translated[role])
        # Base Vehicle.act(None) preserves commands when Road.act runs.
        self._road.act()
        self._road.step(dt_s)
        self._tick += 1
        return LogEntry(
            self.snapshot(),
            tuple(
                AppliedAction(
                    role,
                    ControlAction(
                        float(vehicle.action["acceleration"]),
                        float(vehicle.action["steering"]),
                    ),
                )
                for role, vehicle in self._vehicles.items()
            ),
        )

    def road_exited(self, snapshot: StateSnapshot) -> bool:
        return road_exited(snapshot, self._geometry)

    def backend_settings(self) -> dict[str, str]:
        return {"backend": "highway-env", "backend_version": version("highway-env")}

    def close(self) -> None:
        # Road/Vehicle own no viewer, window, or external resource to tear down.
        self._closed = True
