"""Minimal in-memory simulation history; backend metadata follows in step 2."""

from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise
from math import isclose

from cutin.core._validation import number
from cutin.core.scenario import ScenarioConfig
from cutin.core.state import AppliedAction, Role, Snapshot


class Termination(StrEnum):
    HORIZON = "horizon"
    COLLISION = "collision"
    ROAD_EXIT = "road_exit"
    ERROR = "error"
    CANCELLED = "cancelled"


class EventKind(StrEnum):
    MERGE_START = "merge_start"
    MERGE_COMPLETE = "merge_complete"
    COLLISION = "collision"


@dataclass(frozen=True, slots=True)
class SimulationEvent:
    time_s: float
    kind: EventKind
    roles: tuple[Role, ...]

    def __post_init__(self) -> None:
        number("event time_s", self.time_s, minimum=0)
        if not isinstance(self.kind, EventKind):
            raise ValueError("kind must be an EventKind")
        if (
            not isinstance(self.roles, tuple)
            or not self.roles
            or not all(isinstance(role, Role) for role in self.roles)
            or len(set(self.roles)) != len(self.roles)
        ):
            raise ValueError("event roles must be a nonempty tuple of distinct Roles")


@dataclass(frozen=True, slots=True)
class LogSample:
    """Actions apply to the interval ending at this snapshot; none at t=0."""

    snapshot: Snapshot
    applied_actions: tuple[AppliedAction, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.snapshot, Snapshot):
            raise ValueError("snapshot must be a Snapshot")
        if not isinstance(self.applied_actions, tuple) or not all(
            isinstance(action, AppliedAction) for action in self.applied_actions
        ):
            raise ValueError("applied_actions must be a tuple of AppliedAction records")
        if self.snapshot.time_s == 0:
            if self.applied_actions:
                raise ValueError("initial sample must have no applied actions")
        elif len(self.applied_actions) != 3 or {
            a.role for a in self.applied_actions
        } != set(Role):
            raise ValueError("later samples require one applied action per role")


@dataclass(frozen=True, slots=True)
class SimulationLog:
    """A completed or partial run; actual end time is derived from the last sample.

    This foundation record intentionally does not yet specify backend settings,
    tick spacing, or provenance. The integration spike will establish those.
    """

    scenario: ScenarioConfig
    samples: tuple[LogSample, ...]
    termination: Termination
    events: tuple[SimulationEvent, ...] = ()
    diagnostic: str | None = None
    schema_version: int = 1

    def __post_init__(self) -> None:
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("unsupported log schema_version")
        if not isinstance(self.scenario, ScenarioConfig):
            raise ValueError("scenario must be a ScenarioConfig")
        if not isinstance(self.termination, Termination):
            raise ValueError("termination must be a Termination")
        if (
            not isinstance(self.samples, tuple)
            or not self.samples
            or not all(isinstance(sample, LogSample) for sample in self.samples)
        ):
            raise ValueError("samples must be a nonempty tuple of LogSample records")
        times = tuple(sample.snapshot.time_s for sample in self.samples)
        if times[0] != 0 or any(b <= a for a, b in pairwise(times)):
            raise ValueError("sample times must start at zero and strictly increase")
        if self.end_time_s > self.scenario.duration_s:
            raise ValueError("log extends beyond the scenario horizon")
        if self.termination == Termination.HORIZON and not isclose(
            self.end_time_s, self.scenario.duration_s, rel_tol=0, abs_tol=1e-9
        ):
            raise ValueError("horizon termination must reach the scenario duration")
        if self.termination == Termination.COLLISION and not any(
            vehicle.collided for vehicle in self.samples[-1].snapshot.vehicles
        ):
            raise ValueError("collision termination needs terminal collision evidence")
        if not isinstance(self.events, tuple) or not all(
            isinstance(event, SimulationEvent) for event in self.events
        ):
            raise ValueError("events must be a tuple of SimulationEvent records")
        if any(event.time_s > self.end_time_s for event in self.events):
            raise ValueError("event time must be within the recorded run")
        if self.diagnostic is not None and not isinstance(self.diagnostic, str):
            raise ValueError("diagnostic must be a string or None")
        if self.termination == Termination.ERROR and not self.diagnostic:
            raise ValueError("error termination needs a diagnostic")

    @property
    def end_time_s(self) -> float:
        return self.samples[-1].snapshot.time_s
