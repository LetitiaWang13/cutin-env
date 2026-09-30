"""Bridge our contracts to CutInEnv's joint actions, state, and native renderer."""

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
from cutin.simulation.adapters.cut_in_env import CutInEnv
from cutin.simulation.adapters.highway_env_translation import (
    collect_snapshot,
    road_exited,
    translate_actions,
)


class HighwayEnvAdapter:
    """No SimulatorConfig dependency; the runner supplies dt and generic limits."""

    def __init__(
        self, scenario: ScenarioConfig, *, render_mode: str | None = None
    ) -> None:
        self._geometry = scenario.road
        self._env = CutInEnv(scenario, render_mode=render_mode)

    @property
    def closed(self) -> bool:
        return self._env.done

    @property
    def episode_finished(self) -> bool:
        return self._env.episode_finished

    def snapshot(self) -> StateSnapshot:
        return collect_snapshot(self._env.vehicles_by_role, self._env.steps)

    def step(self, actions: Mapping[Role, ControlAction], dt_s: float) -> LogEntry:
        # Config/unit translation remains outside the native Gym environment.
        # Check dt before translation, which uses it for backend speed bounds.
        if self.closed:
            raise RuntimeError("adapter is closed")
        if not isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        translated = translate_actions(actions, self._env.vehicles_by_role, dt_s)
        self._env.step(translated, dt_s=dt_s)
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
                for role, vehicle in self._env.vehicles_by_role.items()
            ),
        )

    def road_exited(self, snapshot: StateSnapshot) -> bool:
        return road_exited(snapshot, self._geometry)

    def backend_settings(self) -> dict[str, str]:
        return {"backend": "highway-env", "backend_version": version("highway-env")}

    def render(self):
        return self._env.render()

    def close(self) -> None:
        self._env.close()
