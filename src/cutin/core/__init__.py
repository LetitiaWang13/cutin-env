"""Shared domain records; no algorithms, file I/O, or backend imports."""

from cutin.core.log import (
    EventKind,
    LogSample,
    SimulationEvent,
    SimulationLog,
    Termination,
)
from cutin.core.scenario import (
    BaselinePolicy,
    CutInConfig,
    InitialConditions,
    InitialVehicle,
    RoadConfig,
    ScenarioConfig,
    VehicleDimensions,
)
from cutin.core.state import AppliedAction, ControlAction, Role, Snapshot, VehicleState

__all__ = [
    "AppliedAction",
    "BaselinePolicy",
    "ControlAction",
    "CutInConfig",
    "EventKind",
    "InitialConditions",
    "InitialVehicle",
    "LogSample",
    "RoadConfig",
    "Role",
    "ScenarioConfig",
    "SimulationEvent",
    "SimulationLog",
    "Snapshot",
    "Termination",
    "VehicleDimensions",
    "VehicleState",
]
