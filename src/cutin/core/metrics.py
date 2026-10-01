"""Safety result contracts; algorithms belong to evaluation, not core."""

from dataclasses import dataclass
from enum import StrEnum

from cutin.core.log import TerminationReason
from cutin.core.state import Role


class AssessmentStatus(StrEnum):
    COMPLETE = "complete"  # Horizon reached or target collision observed; not "safe".
    NON_TARGET_COLLISION = "non_target_collision"
    INCOMPLETE = "incomplete"


@dataclass(frozen=True)
class PairMetrics:
    """Minima over ego versus all vehicles of one role, not persistent vehicle IDs."""

    other_role: Role
    min_gap_m: float | None = None
    min_gap_tick: int | None = None
    min_ttc_s: float | None = None
    min_ttc_tick: int | None = None


@dataclass(frozen=True)
class Metrics:
    assessment_status: AssessmentStatus
    termination_reason: TerminationReason
    physics_hz: int
    end_tick: int | None
    collision: bool
    ego_collision: bool
    first_collision_tick: int | None
    first_ego_collision_tick: int | None
    collision_roles: tuple[Role, ...]  # Flags at first collision, not inferred pairs.
    pairs: tuple[PairMetrics, ...]
    gap_warning: bool
    ttc_warning: bool
    gap_warning_m: float
    ttc_warning_s: float
    evaluator_version: str = "longitudinal-v1"
    schema_version: int = 1
