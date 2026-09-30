"""Shared records and boundary validation; no file I/O or backend imports."""

from cutin.core.action import AppliedAction, ControlAction
from cutin.core.log import LogEntry, SimulationLog, TerminationReason
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
    "LogEntry",
    "RoadConfig",
    "Role",
    "ScenarioConfig",
    "SimulationLog",
    "Snapshot",
    "TerminationReason",
    "VehicleDimensions",
    "VehicleState",
    "validate_scenario",
]
