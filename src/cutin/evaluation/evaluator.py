"""Evaluate the recorded trajectory of a single simulation."""

from dataclasses import dataclass

from cutin.config import EvaluationConfig
from cutin.core.record import ScenarioRecord
from cutin.evaluation.metrics import lct, ttc
from cutin.evaluation.metrics.robustness import RobustnessResult, from_measurements


@dataclass(frozen=True)
class EvaluationResult:
    """Observed metrics, with collision roles taken from the first collided frame."""

    pairs: tuple[ttc.PairMinima, ttc.PairMinima]
    lane_change_time_s: float | None
    first_collision_tick: int | None
    first_ego_collision_tick: int | None
    collision_roles: tuple[str, ...]
    robustness: RobustnessResult | None = None

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
        """Compute full-episode metrics and score them when targets are provided."""
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

        front, cutin = ttc.compute(record)
        lane_change = lct.observe(record)
        targets = self.config.robustness_targets
        robustness = (
            from_measurements(cutin.min_ttc_s, lane_change, targets)
            if targets is not None
            else None
        )
        return EvaluationResult(
            pairs=(front, cutin),
            lane_change_time_s=lane_change.duration_s,
            first_collision_tick=first_collision,
            first_ego_collision_tick=first_ego_collision,
            collision_roles=collision_roles,
            robustness=robustness,
        )
