"""Concrete inputs for the fixed three-vehicle, three-lane straight-road scene."""

from dataclasses import dataclass

from cutin.config import CutInConfig, IDMConfig


@dataclass
class RoadConfig:
    """Three parallel lanes, directed along world +x, with lane 0 centred at y=0."""

    length_m: float
    lane_width_m: float
    speed_limit_mps: float


@dataclass
class InitialVehicleState:
    """Vehicle-centre x, lane index, and initial speed; initial heading is zero."""

    x_m: float
    lane: int
    speed_mps: float


@dataclass
class ScenarioConfig:
    """One resolved episode; no optimizer ranges or backend-specific objects.

    Relative positions/percentages are resolved into world x before execution.
    The seed belongs to scene execution, not to optimizer candidate sampling.
    """

    road: RoadConfig
    duration_s: float
    seed: int

    ego: InitialVehicleState
    front: InitialVehicleState
    cutin: InitialVehicleState

    ego_idm: IDMConfig
    front_idm: IDMConfig
    cutin_controller: CutInConfig
