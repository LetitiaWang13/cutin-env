"""Four-phase cut-in control on native vehicles sharing one straight road."""

from enum import StrEnum
from math import asin, atan, atan2, tan

import numpy as np
from highway_env import utils
from highway_env.road.road import LaneIndex
from highway_env.vehicle.behavior import IDMVehicle
from highway_env.vehicle.kinematics import Vehicle

from cutin.config import CutInConfig
from cutin.simulation.adapters.highway_env.controllers.idm import IDMController


class CutInPhase(StrEnum):
    WARMUP = "warmup"
    PREPARE = "prepare"
    MERGE = "merge"
    FOLLOW = "follow"


def lateral_reference(
    elapsed_s: float, duration_s: float, source_y_m: float, target_y_m: float
) -> tuple[float, float]:
    """Quintic y/y-rate reference with zero endpoint velocity and acceleration."""
    u = min(max(elapsed_s / duration_s, 0.0), 1.0)
    blend = 10 * u**3 - 15 * u**4 + 6 * u**5
    rate = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration_s
    displacement = target_y_m - source_y_m
    return source_y_m + displacement * blend, displacement * rate


class CutInController:
    """Keep the initial speed until merging finishes, then use native IDM.

    The adapter calls act() on control ticks and update_phase() after physics
    steps. All transitions live in update_phase(); maneuver initiation stays on
    control ticks, while completion is recorded on physics ticks.
    FOLLOW commands begin on the next control tick. Do not call Road.act() as
    well: the cut-in vehicle is a native IDMVehicle whose act() would otherwise
    overwrite the custom warmup/preparation/merge commands.
    """

    def __init__(
        self,
        vehicle: IDMVehicle,
        ego: Vehicle,
        target_lane_index: LaneIndex,
        config: CutInConfig,
    ) -> None:
        self.vehicle = vehicle
        self.ego = ego
        self.config = config
        self.initial_speed_mps = float(vehicle.speed)
        self.source_lane_index = vehicle.lane_index
        self.target_lane_index = target_lane_index
        network = vehicle.road.network
        self.source_y_m = float(
            network.get_lane(self.source_lane_index).position(0, 0)[1]
        )
        self.target_y_m = float(network.get_lane(target_lane_index).position(0, 0)[1])
        self.follow_controller = IDMController(vehicle, config.follow)
        vehicle.target_speed = self.initial_speed_mps
        self.phase = CutInPhase.WARMUP
        self.prepare_start_time_s: float | None = None
        self.merge_start_time_s: float | None = None
        self.merge_completion_time_s: float | None = None
        self._previous_gap_error_m: float | None = None

    @property
    def relative_gap_m(self) -> float:
        """Longitudinal bumper gap used for preparation, even in adjacent lanes."""
        return float(
            self.vehicle.position[0]
            - self.ego.position[0]
            - (self.vehicle.LENGTH + self.ego.LENGTH) / 2
        )

    def act(self, time_s: float) -> None:
        """Update the phase, then issue the action for that phase."""
        if self.vehicle.crashed:
            return
        self.update_phase(time_s, control_tick=True)

        match self.phase:
            case CutInPhase.WARMUP | CutInPhase.PREPARE:
                steering = self.vehicle.steering_control(self.source_lane_index)
            case CutInPhase.MERGE:
                y_m, y_rate_mps = lateral_reference(
                    time_s - self.merge_start_time_s,
                    self.config.lane_change_duration_s,
                    self.source_y_m,
                    self.target_y_m,
                )
                steering = self._track_reference(y_m, y_rate_mps)
            case CutInPhase.FOLLOW:
                self.follow_controller.act()
                return

        self._apply_constant_speed_action(steering)

    def update_phase(self, time_s: float, *, control_tick: bool = False) -> None:
        """Update the state machine without generating actions or advancing physics.

        act() passes control_tick=True so preparation and merging start alongside
        control decisions. After Road.step(), the adapter uses the default to
        observe physical completion without starting a maneuver between decisions.
        """
        if self.vehicle.crashed:
            return

        if control_tick:
            if self.phase == CutInPhase.WARMUP and time_s >= self.config.warmup_s:
                self.phase = CutInPhase.PREPARE
                self.prepare_start_time_s = time_s
            if self.phase == CutInPhase.PREPARE:
                error = self.relative_gap_m - self.config.target_gap_m
                crossed = (
                    self._previous_gap_error_m is not None
                    and self._previous_gap_error_m * error < 0
                )
                if abs(error) <= self.config.trigger_tolerance_m or crossed:
                    self.phase = CutInPhase.MERGE
                    self.merge_start_time_s = time_s
                self._previous_gap_error_m = error

        if self.phase == CutInPhase.MERGE:
            lane = self.vehicle.road.network.get_lane(self.target_lane_index)
            longitudinal, lateral = lane.local_coordinates(self.vehicle.position)
            if (
                time_s - self.merge_start_time_s >= self.config.lane_change_duration_s
                and abs(lateral) <= self.config.completion_position_tolerance_m
                and abs(lane.local_angle(self.vehicle.heading, longitudinal))
                <= self.config.completion_heading_tolerance_rad
            ):
                self.phase = CutInPhase.FOLLOW
                self.merge_completion_time_s = time_s
                self.vehicle.target_lane_index = self.target_lane_index
                self.vehicle.target_speed = self.config.follow.desired_speed_mps

    def _apply_constant_speed_action(self, steering: float) -> None:
        """Hold the initial speed and apply steering without invoking native IDM."""
        Vehicle.act(
            self.vehicle,
            {
                "acceleration": float(
                    np.clip(
                        self.vehicle.speed_control(self.initial_speed_mps),
                        -self.vehicle.ACC_MAX,
                        self.vehicle.ACC_MAX,
                    )
                ),
                "steering": float(
                    np.clip(
                        steering,
                        -self.vehicle.MAX_STEERING_ANGLE,
                        self.vehicle.MAX_STEERING_ANGLE,
                    )
                ),
            },
        )

    def _track_reference(self, y_m: float, y_rate_mps: float) -> float:
        """Lateral feedforward plus feedback, inverted through the bicycle model."""
        vehicle = self.vehicle
        if vehicle.speed <= 0.1:
            return 0.0  # Steering cannot move a stopped bicycle model sideways.
        heading = atan2(
            y_rate_mps + vehicle.KP_LATERAL * (y_m - vehicle.position[1]),
            vehicle.speed,
        )
        heading_rate = vehicle.KP_HEADING * utils.wrap_to_pi(heading - vehicle.heading)
        slip = asin(
            float(np.clip(vehicle.LENGTH * heading_rate / (2 * vehicle.speed), -1, 1))
        )
        return atan(2 * tan(slip))
