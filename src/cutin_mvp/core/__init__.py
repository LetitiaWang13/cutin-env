"""Shared records and boundary validation; no file I/O or backend imports."""

from cutin_mvp.core.action import AppliedAction, ControlAction
from cutin_mvp.core.log import CutInReport, LogEntry, SimulationLog, TerminationReason
from cutin_mvp.core.metrics import AssessmentStatus, Metrics, PairMetrics
from cutin_mvp.core.scenario_config import (
    BaselinePolicy,
    CutInConfig,
    InitialConditions,
    InitialVehicle,
    RoadConfig,
    ScenarioConfig,
    VehicleDimensions,
    validate_scenario,
)
from cutin_mvp.core.state import Role, StateSnapshot, VehicleState

__all__ = [
    "AppliedAction",
    "AssessmentStatus",
    "BaselinePolicy",
    "ControlAction",
    "CutInConfig",
    "CutInReport",
    "InitialConditions",
    "InitialVehicle",
    "LogEntry",
    "Metrics",
    "PairMetrics",
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
