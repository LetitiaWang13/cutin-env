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
    """Vehicle states at one tick; seconds = tick / physics_hz.

    Producers supply a tuple. Vehicle count and role uniqueness are unrestricted.
    """

    tick: int
    vehicles: tuple[VehicleState, ...]

    def vehicles_for_role(self, role: Role) -> tuple[VehicleState, ...]:
        """Return all matching states in snapshot order, or an empty tuple."""
        return tuple(vehicle for vehicle in self.vehicles if vehicle.role == role)
