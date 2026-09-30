"""Normalized state records. Simulation owns their validity and integer clock."""

from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType


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
    """Role-grouped states at one tick; seconds = tick / physics_hz.

    Each role may contain multiple vehicles. Copy and freeze the containers so
    later producer updates cannot alter history. No domain validation here.
    """

    tick: int
    vehicles: Mapping[Role, tuple[VehicleState, ...]]

    def __post_init__(self) -> None:
        grouped = {role: tuple(states) for role, states in self.vehicles.items()}
        object.__setattr__(self, "vehicles", MappingProxyType(grouped))
