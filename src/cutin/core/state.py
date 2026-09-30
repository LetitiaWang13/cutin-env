"""Normalized state records. Simulation owns their validity and integer clock."""

from dataclasses import dataclass
from enum import StrEnum


class Role(StrEnum):
    EGO = "ego"
    FRONT = "front"
    CUT_IN = "cut_in"


@dataclass(frozen=True)
class VehicleState:
    role: Role
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    length_m: float
    width_m: float
    collided: bool = False


@dataclass(frozen=True)
class StateSnapshot:
    """Vehicle states at one tick; time in seconds is tick / physics_hz."""

    tick: int
    vehicles: tuple[VehicleState, ...]

    def vehicle(self, role: Role) -> VehicleState:
        for vehicle in self.vehicles:
            if vehicle.role == role:
                return vehicle
        raise KeyError(role)
