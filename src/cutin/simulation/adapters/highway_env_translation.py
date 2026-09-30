"""Translate scenario, action, and state contracts at the highway-env boundary."""

from collections.abc import Mapping
from math import cos, isfinite, sin

import numpy as np
from highway_env.road.lane import StraightLane
from highway_env.road.road import Road, RoadNetwork
from highway_env.vehicle.kinematics import Vehicle

from cutin.core import ControlAction, Role, ScenarioConfig, StateSnapshot, VehicleState
from cutin.core.scenario_config import RoadConfig


def build_scene(scenario: ScenarioConfig) -> tuple[Road, dict[Role, Vehicle]]:
    """Translate a validated scenario into backend objects, not stock env.config.

    Generic scenario validation belongs to the execution/input boundary. These
    checks concern only features supported by this backend integration.
    """
    if (scenario.vehicles.length_m, scenario.vehicles.width_m) != (
        Vehicle.LENGTH,
        Vehicle.WIDTH,
    ):
        raise ValueError("highway-env currently supports only 5 m x 2 m vehicles")
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
        raise ValueError(f"highway-env supports speeds up to {Vehicle.MAX_SPEED} m/s")
    network = RoadNetwork()
    for lane in range(scenario.road.lane_count):
        y = lane * scenario.road.lane_width_m
        network.add_lane(
            "start",
            "end",
            StraightLane(
                [0, y], [scenario.road.length_m, y], width=scenario.road.lane_width_m
            ),
        )
    road = Road(network=network, np_random=np.random.default_rng(scenario.seed))
    vehicles = {
        role: Vehicle(
            road,
            [state.x_m, state.lane * scenario.road.lane_width_m],
            speed=state.speed_mps,
        )
        for role, state in initial.items()
    }
    road.vehicles = list(vehicles.values())
    return road, vehicles


def translate_actions(
    actions: Mapping[Role, ControlAction], vehicles: Mapping[Role, Vehicle], dt_s: float
) -> dict[Role, dict[str, float]]:
    """Map units/fields and enforce backend speed limits, not policy limits.

    The runner bounds generic acceleration/braking/steering requests. This
    translation prevents speed overshoot/reversing within a physics step.
    """
    if set(actions) != set(vehicles):
        raise ValueError("provide one action for each initialized vehicle role")
    translated = {}
    for role, vehicle in vehicles.items():
        action = actions[role]
        if not isfinite(action.acceleration_mps2) or not isfinite(action.steering_rad):
            raise ValueError("actions must be finite")
        translated[role] = {
            "acceleration": min(
                max(action.acceleration_mps2, -vehicle.speed / dt_s),
                (Vehicle.MAX_SPEED - vehicle.speed) / dt_s,
            ),
            "steering": action.steering_rad,
        }
    return translated


def collect_snapshot(vehicles: Mapping[Role, Vehicle], tick: int) -> StateSnapshot:
    """Copy mutable backend state into our immutable records."""
    return StateSnapshot(
        tick=tick,
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
            for role, vehicle in vehicles.items()
        ),
    )


def road_exited(snapshot: StateSnapshot, road: RoadConfig) -> bool:
    """Conservative rotated footprint bounds, not a backend lane-centre test."""
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
