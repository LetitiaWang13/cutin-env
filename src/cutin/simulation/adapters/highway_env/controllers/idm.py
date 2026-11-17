"""Per-vehicle configuration and delegation to highway-env's native IDM."""

from highway_env.vehicle.behavior import IDMVehicle

from cutin.config import IDMConfig


class IDMController:
    """Configure an existing native vehicle without replacing its dynamics.

    The adapter calls act() only on control ticks, then advances Road.step().
    It must not also call Road.act(), which would repeat native decisions.
    """

    def __init__(self, vehicle: IDMVehicle, config: IDMConfig) -> None:
        self.vehicle = vehicle
        self.config = config
        vehicle.target_speed = config.desired_speed_mps
        vehicle.enable_lane_change = False
        vehicle.TIME_WANTED = config.time_headway_s
        vehicle.DISTANCE_WANTED = config.minimum_gap_m + vehicle.LENGTH
        vehicle.COMFORT_ACC_MAX = config.comfortable_acceleration_mps2
        vehicle.COMFORT_ACC_MIN = -config.comfortable_braking_mps2
        vehicle.ACC_MAX = config.acceleration_limit_mps2
        vehicle.DELTA = config.speed_exponent

    def act(self) -> None:
        self.vehicle.act()
