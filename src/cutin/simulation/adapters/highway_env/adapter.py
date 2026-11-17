"""Headless native-road execution: one concrete scenario in, one motion record out."""

from importlib.metadata import version
from math import ceil

import numpy as np
from highway_env.road.lane import StraightLane
from highway_env.road.road import Road, RoadNetwork
from highway_env.vehicle.behavior import IDMVehicle

from cutin.core.record import Frame, ScenarioRecord, VehicleState
from cutin.core.scenario import InitialVehicleState, ScenarioConfig
from cutin.simulation.adapters.highway_env.controllers.cutin import (
    CutInController,
    CutInPhase,
)
from cutin.simulation.adapters.highway_env.controllers.idm import IDMController
from cutin.simulation.interface import Simulator

# Bump when custom controller behavior changes; native IDM is versioned by backend.
CONTROLLER_VERSION = "native-idm+cutin-v1"


class HighwayEnvAdapter(Simulator):
    def run(self, scenario: ScenarioConfig) -> ScenarioRecord:
        """Execute fresh road/vehicle/controller state without rendering or file I/O.

        Control rates are integer divisors of physics_hz. A non-integral horizon
        ends on the first physics tick at or beyond duration_s. Backend exceptions
        propagate; this slice has no error-recovery or input-validation framework.
        """
        road, vehicles = _create_scene(scenario)
        ego, front, cutin = vehicles
        ego_controller = IDMController(ego, scenario.ego_idm)
        front_controller = IDMController(front, scenario.front_idm)
        cutin_controller = CutInController(
            cutin,
            ego,
            ("start", "end", scenario.ego.lane),
            scenario.cutin_controller,
        )
        physics_hz = self.config.physics_hz
        control_interval = physics_hz // self.config.controller_hz
        horizon_ticks = ceil(scenario.duration_s * physics_hz)
        frames = [_capture_frame(0, vehicles, cutin_controller.phase)]
        termination_reason = "horizon"

        for tick in range(1, horizon_ticks + 1):
            if (tick - 1) % control_interval == 0:
                ego_controller.act()
                front_controller.act()
                cutin_controller.act((tick - 1) / physics_hz)
            road.step(1 / physics_hz)
            cutin_controller.update_phase(tick / physics_hz)
            frames.append(_capture_frame(tick, vehicles, cutin_controller.phase))
            if any(vehicle.crashed for vehicle in vehicles):
                termination_reason = "collision"
                break

        return ScenarioRecord(
            scenario=scenario,
            simulation=self.config,
            backend_version=version("highway-env"),
            controller_version=CONTROLLER_VERSION,
            frames=frames,
            termination_reason=termination_reason,
            prepare_start_time_s=cutin_controller.prepare_start_time_s,
            merge_start_time_s=cutin_controller.merge_start_time_s,
            merge_completion_time_s=cutin_controller.merge_completion_time_s,
        )


def _create_scene(
    scenario: ScenarioConfig,
) -> tuple[Road, tuple[IDMVehicle, IDMVehicle, IDMVehicle]]:
    """Build exactly three parallel lanes and the three explicitly named vehicles."""
    network = RoadNetwork()
    for lane in range(3):
        y_m = lane * scenario.road.lane_width_m
        network.add_lane(
            "start",
            "end",
            StraightLane(
                [0, y_m],
                [scenario.road.length_m, y_m],
                width=scenario.road.lane_width_m,
                speed_limit=scenario.road.speed_limit_mps,
            ),
        )
    road = Road(network=network, np_random=np.random.default_rng(scenario.seed))

    def create_vehicle(initial: InitialVehicleState) -> IDMVehicle:
        lane_index = ("start", "end", initial.lane)
        lane = network.get_lane(lane_index)
        return IDMVehicle(
            road,
            lane.position(initial.x_m, 0),
            speed=initial.speed_mps,
            target_lane_index=lane_index,
            enable_lane_change=False,
        )

    vehicles = (
        create_vehicle(scenario.ego),
        create_vehicle(scenario.front),
        create_vehicle(scenario.cutin),
    )
    road.vehicles = list(vehicles)
    return road, vehicles


def _capture_frame(
    tick: int,
    vehicles: tuple[IDMVehicle, IDMVehicle, IDMVehicle],
    phase: CutInPhase,
) -> Frame:
    ego, front, cutin = vehicles
    return Frame(
        tick=tick,
        ego=_vehicle_state(ego, initial=tick == 0),
        front=_vehicle_state(front, initial=tick == 0),
        cutin=_vehicle_state(cutin, initial=tick == 0),
        cutin_phase=phase.value,
    )


def _vehicle_state(vehicle: IDMVehicle, *, initial: bool) -> VehicleState:
    """Detach all values from mutable backend arrays and held action dictionaries."""
    return VehicleState(
        x_m=float(vehicle.position[0]),
        y_m=float(vehicle.position[1]),
        heading_rad=float(vehicle.heading),
        speed_mps=float(vehicle.speed),
        length_m=float(vehicle.LENGTH),
        width_m=float(vehicle.WIDTH),
        collided=bool(vehicle.crashed),
        acceleration_mps2=None if initial else float(vehicle.action["acceleration"]),
        steering_rad=None if initial else float(vehicle.action["steering"]),
    )
