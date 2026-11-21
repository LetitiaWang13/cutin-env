"""Evaluate the recorded trajectory of a single simulation."""

from dataclasses import dataclass

from cutin.config import EvaluationConfig
from cutin.core.record import ScenarioRecord
from cutin.evaluation.metrics import lct, ttc


@dataclass(frozen=True)
class EvaluationResult:
    """Observed metrics, with collision roles taken from the first collided frame."""

    pairs: tuple[ttc.PairMinima, ttc.PairMinima]
    lane_change_time_s: float | None
    first_collision_tick: int | None
    first_ego_collision_tick: int | None
    collision_roles: tuple[str, ...]

    @property
    def collision(self) -> bool:
        return self.first_collision_tick is not None

    @property
    def ego_collision(self) -> bool:
        return self.first_ego_collision_tick is not None


class Evaluator:
    def __init__(self, config: EvaluationConfig) -> None:
        self.config = config

    def evaluate(self, record: ScenarioRecord) -> EvaluationResult:
        """Compute full-episode metrics. The initial metrics need no settings."""
        first_collision = first_ego_collision = None
        collision_roles = ()
        for frame in record.frames:
            collided = tuple(
                role
                for role in ("ego", "front", "cutin")
                if getattr(frame, role).collided
            )
            if collided and first_collision is None:
                first_collision = frame.tick
                collision_roles = collided
            if frame.ego.collided:
                first_ego_collision = frame.tick
                break

        return EvaluationResult(
            pairs=ttc.compute(record),
            lane_change_time_s=lct.compute(record),
            first_collision_tick=first_collision,
            first_ego_collision_tick=first_ego_collision,
            collision_roles=collision_roles,
        )
