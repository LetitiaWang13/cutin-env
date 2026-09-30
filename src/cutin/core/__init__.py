"""Shared records and boundary validation; no file I/O or backend imports."""

from cutin.core.action import AppliedAction, ControlAction
from cutin.core.log import LogSample, SimulationLog, Termination
from cutin.core.scenario import (
    BaselinePolicy,
    CutInConfig,
    InitialConditions,
    InitialVehicle,
    RoadConfig,
    ScenarioConfig,
    VehicleDimensions,
    validate_scenario,
)
from cutin.core.state import Role, Snapshot, VehicleState

__all__ = [
    "AppliedAction",
    "BaselinePolicy",
    "ControlAction",
    "CutInConfig",
    "InitialConditions",
    "InitialVehicle",
    "LogSample",
    "RoadConfig",
    "Role",
    "ScenarioConfig",
    "SimulationLog",
    "Snapshot",
    "Termination",
    "VehicleDimensions",
    "VehicleState",
    "validate_scenario",
]
