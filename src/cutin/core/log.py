"""Minimal run history. The runner, not record constructors, owns consistency."""

from dataclasses import dataclass
from enum import StrEnum

from cutin.core.action import AppliedAction
from cutin.core.scenario import ScenarioConfig
from cutin.core.state import Snapshot


class Termination(StrEnum):
    HORIZON = "horizon"
    COLLISION = "collision"
    ROAD_EXIT = "road_exit"
    ERROR = "error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class LogSample:
    """Actions apply to the interval ending here; the initial sample has none."""

    snapshot: Snapshot
    applied_actions: tuple[AppliedAction, ...] = ()


@dataclass(frozen=True)
class SimulationLog:
    """Resolved input, sampled motion, and termination. No event framework yet.

    Backend settings and serialization will follow the integration probe.
    """

    scenario: ScenarioConfig
    samples: tuple[LogSample, ...]
    termination: Termination
    diagnostic: str | None = None
