"""Small baseline policies, independent of backend objects. Defaults uncalibrated."""

from math import asin, atan, atan2, cos, pi, sin, sqrt, tan

from cutin.core import (
    ControlAction,
    CutInReport,
    Role,
    ScenarioConfig,
    StateSnapshot,
    VehicleState,
)
from cutin.core.scenario_config import BaselinePolicy
from cutin.simulation.config import SimulatorConfig

# Experimental bicycle-model tracking gains/tolerances; versioned as baseline-v1.
POSITION_GAIN = 1.0  # 1/s
HEADING_GAIN = 3.0  # 1/s
SPEED_GAIN = 1.0  # 1/s
COMPLETION_POSITION_M = 0.2
COMPLETION_HEADING_RAD = 0.05


def half_extents(vehicle: VehicleState) -> tuple[float, float]:
    """Heading-aware, conservative world-axis rectangle bounds."""
    c, s = abs(cos(vehicle.heading_rad)), abs(sin(vehicle.heading_rad))
    return (
        (vehicle.length_m * c + vehicle.width_m * s) / 2,
        (vehicle.length_m * s + vehicle.width_m * c) / 2,
    )


def following_acceleration(
    vehicle: VehicleState, snapshot: StateSnapshot, policy: BaselinePolicy
) -> float:
    """Bounded IDM-style following, including partially intruding vehicles.

    Uses projected bumper gap and constant longitudinal closing speed. This is
    an experimental baseline, not a guarantee of collision avoidance.
    """
    speed = vehicle.speed_mps
    if policy.desired_speed_mps == 0:
        acceleration = -policy.max_braking_mps2 if speed > 0 else 0.0
    else:
        acceleration = policy.max_acceleration_mps2 * (
            1 - (speed / policy.desired_speed_mps) ** 4
        )
    own_x, own_y = half_extents(vehicle)
    leaders = []
    for other in snapshot.vehicles:
        # Compare identity, not role: other vehicles may share a role.
        if other is vehicle or other.x_m < vehicle.x_m:
            continue
        other_x, other_y = half_extents(other)
        if abs(other.y_m - vehicle.y_m) <= own_y + other_y:
            gap = other.x_m - vehicle.x_m - own_x - other_x
            closing = speed * cos(vehicle.heading_rad) - other.speed_mps * cos(
                other.heading_rad
            )
            leaders.append((gap, closing))
    if leaders:
        gap, closing = min(leaders, key=lambda pair: pair[0])
        if gap <= policy.minimum_gap_m:
            return -policy.max_braking_mps2
        desired_gap = policy.minimum_gap_m + max(
            0.0,
            speed * policy.time_headway_s
            + speed
            * closing
            / (2 * sqrt(policy.max_acceleration_mps2 * policy.max_braking_mps2)),
        )
        acceleration -= policy.max_acceleration_mps2 * (desired_gap / gap) ** 2
    return min(
        max(acceleration, -policy.max_braking_mps2), policy.max_acceleration_mps2
    )


def track_lateral(vehicle: VehicleState, y_m: float, y_rate_mps: float) -> float:
    """Bicycle-model steering request; the runner applies physical actuator bounds."""
    if vehicle.speed_mps <= 0.1:
        return 0.0
    desired_heading = atan2(
        y_rate_mps + POSITION_GAIN * (y_m - vehicle.y_m), vehicle.speed_mps
    )
    error = (desired_heading - vehicle.heading_rad + pi) % (2 * pi) - pi
    sine_beta = HEADING_GAIN * error * vehicle.length_m / (2 * vehicle.speed_mps)
    beta = asin(min(max(sine_beta, -0.99), 0.99))
    return atan(2 * tan(beta))


def merge_reference(
    elapsed_s: float, duration_s: float, lane_width_m: float
) -> tuple[float, float]:
    """Quintic lateral reference from lane 0 to lane 1, not direct position edits."""
    u = min(max(elapsed_s / duration_s, 0.0), 1.0)
    blend = 10 * u**3 - 15 * u**4 + 6 * u**5
    rate = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration_s
    return lane_width_m * blend, lane_width_m * rate


class DrivingPolicy:
    """Fresh per run. WAIT -> MERGE -> FOLLOW; position preparation is deferred."""

    def __init__(self, scenario: ScenarioConfig, config: SimulatorConfig) -> None:
        self.scenario = scenario
        self.config = config
        self.start_tick: int | None = None
        self.completion_tick: int | None = None

    def act(self, snapshot: StateSnapshot) -> dict[Role, ControlAction]:
        time_s = snapshot.tick / self.config.physics_hz
        (ego,) = snapshot.vehicles_for_role(Role.EGO)
        (front,) = snapshot.vehicles_for_role(Role.FRONT)
        (cut_in,) = snapshot.vehicles_for_role(Role.CUT_IN)
        behavior = self.scenario.cut_in
        width = self.scenario.road.lane_width_m
        if self.start_tick is None and time_s >= behavior.start_time_s:
            self.start_tick = snapshot.tick
        if self.start_tick is None:
            y, rate = 0.0, 0.0
        else:
            elapsed = (snapshot.tick - self.start_tick) / self.config.physics_hz
            y, rate = merge_reference(elapsed, behavior.lane_change_duration_s, width)
        acceleration = (
            following_acceleration(cut_in, snapshot, self.scenario.baseline_policy)
            if self.completion_tick is not None
            else SPEED_GAIN * (behavior.target_speed_mps - cut_in.speed_mps)
        )
        actions = {
            vehicle.role: ControlAction(
                following_acceleration(
                    vehicle, snapshot, self.scenario.baseline_policy
                ),
                track_lateral(vehicle, width, 0),
            )
            for vehicle in (ego, front)
        }
        actions[Role.CUT_IN] = ControlAction(
            acceleration, track_lateral(cut_in, y, rate)
        )
        return actions

    def observe(self, snapshot: StateSnapshot) -> None:
        """Record actual completion at physics rate; FOLLOW starts next control tick."""
        if self.start_tick is None or self.completion_tick is not None:
            return
        elapsed = (snapshot.tick - self.start_tick) / self.config.physics_hz
        (cut_in,) = snapshot.vehicles_for_role(Role.CUT_IN)
        heading_error = (cut_in.heading_rad + pi) % (2 * pi) - pi
        if (
            elapsed >= self.scenario.cut_in.lane_change_duration_s
            and abs(cut_in.y_m - self.scenario.road.lane_width_m)
            <= COMPLETION_POSITION_M
            and abs(heading_error) <= COMPLETION_HEADING_RAD
            and not cut_in.collided
        ):
            self.completion_tick = snapshot.tick

    def report(self) -> CutInReport:
        return CutInReport(self.start_tick, self.completion_tick)
