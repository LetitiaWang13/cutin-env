"""Minimal run history. The runner, not record constructors, owns consistency."""

from dataclasses import dataclass, field
from enum import StrEnum

from cutin.core.action import AppliedAction
from cutin.core.scenario_config import ScenarioConfig
from cutin.core.state import StateSnapshot


class TerminationReason(StrEnum):
    HORIZON = "horizon"
    COLLISION = "collision"
    ROAD_EXIT = "road_exit"
    ERROR = "error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class LogEntry:
    """One snapshot and its interval's applied actions; the initial entry has none."""

    snapshot: StateSnapshot
    applied_actions: tuple[AppliedAction, ...] = ()


@dataclass(frozen=True)
class CutInReport:
    """Actual reference start/completion ticks; None means not observed."""

    start_tick: int | None = None
    completion_tick: int | None = None


@dataclass(frozen=True)
class SimulationLog:
    """Resolved input, sampled motion, and termination. No event framework yet.

    Simulator settings are plain metadata, not a dependency on backend types.
    """

    scenario: ScenarioConfig
    entries: tuple[LogEntry, ...]
    termination_reason: TerminationReason
    diagnostic: str | None = None
    simulator_settings: dict[str, str | int | float] = field(default_factory=dict)
    cut_in_report: CutInReport | None = None
