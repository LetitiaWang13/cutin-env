"""Shared records and boundary validation; no file I/O or backend imports."""

from cutin.core.action import AppliedAction, ControlAction
from cutin.core.log import CutInReport, LogEntry, SimulationLog, TerminationReason
from cutin.core.scenario_config import (
    BaselinePolicy,
    CutInConfig,
    InitialConditions,
    InitialVehicle,
    RoadConfig,
    ScenarioConfig,
    VehicleDimensions,
    validate_scenario,
)
from cutin.core.state import Role, StateSnapshot, VehicleState

__all__ = [
    "AppliedAction",
    "BaselinePolicy",
    "ControlAction",
    "CutInConfig",
    "CutInReport",
    "InitialConditions",
    "InitialVehicle",
    "LogEntry",
    "RoadConfig",
    "Role",
    "ScenarioConfig",
    "SimulationLog",
    "StateSnapshot",
    "TerminationReason",
    "VehicleDimensions",
    "VehicleState",
    "validate_scenario",
]
