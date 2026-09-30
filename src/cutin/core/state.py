"""Immutable normalized state and action records, expressed in SI units."""

from dataclasses import dataclass
from enum import StrEnum

from cutin.core._validation import number, positive


class Role(StrEnum):
    EGO = "ego"
    FRONT = "front"
    CUT_IN = "cut_in"


@dataclass(frozen=True, slots=True)
class VehicleState:
    role: Role
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    length_m: float
    width_m: float
    collided: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.role, Role):
            raise ValueError("role must be a Role")
        for name in ("x_m", "y_m", "heading_rad"):
            number(name, getattr(self, name))
        number("speed_mps", self.speed_mps, minimum=0)
        positive("length_m", self.length_m)
        positive("width_m", self.width_m)
        if type(self.collided) is not bool:
            raise ValueError("collided must be a boolean")


@dataclass(frozen=True, slots=True)
class Snapshot:
    time_s: float
    vehicles: tuple[VehicleState, ...]

    def __post_init__(self) -> None:
        number("time_s", self.time_s, minimum=0)
        if not isinstance(self.vehicles, tuple) or not all(
            isinstance(vehicle, VehicleState) for vehicle in self.vehicles
        ):
            raise ValueError("vehicles must be a tuple of VehicleState records")
        if len(self.vehicles) != 3 or {v.role for v in self.vehicles} != set(Role):
            raise ValueError(
                "snapshot must contain each of the three roles exactly once"
            )

    def vehicle(self, role: Role) -> VehicleState:
        if not isinstance(role, Role):
            raise ValueError("role must be a Role")
        return next(vehicle for vehicle in self.vehicles if vehicle.role == role)


@dataclass(frozen=True, slots=True)
class ControlAction:
    acceleration_mps2: float
    steering_rad: float

    def __post_init__(self) -> None:
        number("acceleration_mps2", self.acceleration_mps2)
        number("steering_rad", self.steering_rad)


@dataclass(frozen=True, slots=True)
class AppliedAction:
    """Role-keyed action; adapters log the bounded value actually applied."""

    role: Role
    action: ControlAction

    def __post_init__(self) -> None:
        if not isinstance(self.role, Role):
            raise ValueError("role must be a Role")
        if not isinstance(self.action, ControlAction):
            raise ValueError("action must be a ControlAction")
