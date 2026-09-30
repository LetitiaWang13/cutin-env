"""Headless highway-env primitives, without stock traffic, rewards, or renderer."""

from collections.abc import Mapping
from importlib.metadata import version
from math import cos, isclose, isfinite, sin

import numpy as np
from highway_env.road.lane import StraightLane
from highway_env.road.road import Road, RoadNetwork
from highway_env.vehicle.kinematics import Vehicle

from cutin.core import (
    AppliedAction,
    ControlAction,
    LogEntry,
    Role,
    ScenarioConfig,
    StateSnapshot,
    VehicleState,
    validate_scenario,
)
from cutin.simulation.config import SimulatorConfig, validate_simulator_config


class HighwayAdapter:
    """One physics tick per step; a future runner owns policies and termination.

    Exactly three unique roles belong to this MVP adapter, not to StateSnapshot.
    Snapshots never contain backend objects. Fresh adapters have fresh state.
    """

    def __init__(self, scenario: ScenarioConfig, config: SimulatorConfig) -> None:
        validate_scenario(scenario)
        validate_simulator_config(config)
        ticks = scenario.duration_s * config.physics_hz
        if (
            not isfinite(ticks)
            or ticks < 1
            or not isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8)
        ):
            raise ValueError("duration_s must span an integral number of physics ticks")
        if (scenario.vehicles.length_m, scenario.vehicles.width_m) != (
            Vehicle.LENGTH,
            Vehicle.WIDTH,
        ):
            raise ValueError(
                "highway adapter currently supports only 5 m x 2 m vehicles"
            )
        initial = {
            Role.EGO: scenario.initial.ego,
            Role.FRONT: scenario.initial.front,
            Role.CUT_IN: scenario.initial.cut_in,
        }
        speeds = [state.speed_mps for state in initial.values()]
        speeds += [
            scenario.cut_in.target_speed_mps,
            scenario.baseline_policy.desired_speed_mps,
        ]
        if any(speed > Vehicle.MAX_SPEED for speed in speeds):
            raise ValueError(
                f"highway adapter supports speeds up to {Vehicle.MAX_SPEED} m/s"
            )

        self.scenario = scenario
        self.config = config
        self._tick = 0
        self._closed = False
        network = RoadNetwork()
        for lane in range(scenario.road.lane_count):
            y = lane * scenario.road.lane_width_m
            network.add_lane(
                "start",
                "end",
                StraightLane(
                    [0, y],
                    [scenario.road.length_m, y],
                    width=scenario.road.lane_width_m,
                ),
            )
        self._road = Road(
            network=network, np_random=np.random.default_rng(scenario.seed)
        )
        self._vehicles = {
            role: Vehicle(
                self._road,
                [state.x_m, state.lane * scenario.road.lane_width_m],
                speed=state.speed_mps,
            )
            for role, state in initial.items()
        }
        self._road.vehicles = list(self._vehicles.values())

    def snapshot(self) -> StateSnapshot:
        return StateSnapshot(
            tick=self._tick,
            vehicles=tuple(
                VehicleState(
                    role=role,
                    x_m=float(vehicle.position[0]),
                    y_m=float(vehicle.position[1]),
                    heading_rad=float(vehicle.heading),
                    speed_mps=float(vehicle.speed),
                    length_m=float(vehicle.LENGTH),
                    width_m=float(vehicle.WIDTH),
                    collided=bool(vehicle.crashed),
                )
                for role, vehicle in self._vehicles.items()
            ),
        )

    def step(self, actions: Mapping[Role, ControlAction]) -> LogEntry:
        if self._closed:
            raise RuntimeError("adapter is closed")
        if set(actions) != set(self._vehicles):
            raise ValueError("provide one action for each initialized vehicle role")
        dt = 1 / self.config.physics_hz
        bounded = {}
        for role, vehicle in self._vehicles.items():
            action = actions[role]
            if not isfinite(action.acceleration_mps2) or not isfinite(
                action.steering_rad
            ):
                raise ValueError("actions must be finite")
            # Prevent reverse motion/speed overshoot and respect actuator bounds.
            acceleration = min(
                max(
                    action.acceleration_mps2,
                    -self.config.max_braking_mps2,
                    -vehicle.speed / dt,
                ),
                self.config.max_acceleration_mps2,
                (Vehicle.MAX_SPEED - vehicle.speed) / dt,
            )
            steering = min(
                max(action.steering_rad, -self.config.max_steering_rad),
                self.config.max_steering_rad,
            )
            bounded[role] = {"acceleration": acceleration, "steering": steering}
        for role, vehicle in self._vehicles.items():
            vehicle.act(bounded[role])
        # Base Vehicle.act(None) preserves external commands when Road.act runs.
        self._road.act()
        self._road.step(dt)
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
        """Conservative rotated footprint bounds, not highway's lane-centre check."""
        road = self.scenario.road
        for state in snapshot.vehicles:
            c, s = abs(cos(state.heading_rad)), abs(sin(state.heading_rad))
            half_x = (state.length_m * c + state.width_m * s) / 2
            half_y = (state.length_m * s + state.width_m * c) / 2
            if (
                state.x_m - half_x < 0
                or state.x_m + half_x > road.length_m
                or state.y_m - half_y < -road.lane_width_m / 2
                or state.y_m + half_y > (road.lane_count - 0.5) * road.lane_width_m
            ):
                return True
        return False

    def simulator_settings(self) -> dict[str, str | int | float]:
        return {**self.config.to_dict(), "backend_version": version("highway-env")}

    def close(self) -> None:
        # Road/Vehicle own no viewer, window, or external resource to tear down.
        self._closed = True
