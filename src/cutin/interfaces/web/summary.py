"""Format recorded evaluation metrics and apply Web display thresholds."""

from dataclasses import asdict, dataclass

from cutin.config import EvaluationConfig
from cutin.core.record import ScenarioRecord
from cutin.evaluation.evaluator import Evaluator
from cutin.evaluation.metrics.ttc import observe_pair as observe_pair

# Names shown in the page; the record's own field for the cut-in vehicle is `cutin`.
LABELS = {"ego": "ego", "front": "front", "cutin": "cut_in"}


@dataclass(frozen=True)
class Warnings:
    gap_warning_m: float
    ttc_warning_s: float


def summarize(record: ScenarioRecord, warnings: Warnings) -> dict:
    frames = record.frames
    result = Evaluator(EvaluationConfig()).evaluate(record)
    if record.termination_reason == "collision":
        status = "complete" if result.ego_collision else "non_target_collision"
    elif record.termination_reason == "horizon" and not result.collision:
        status = "complete"
    else:
        status = "incomplete"
    pairs = [asdict(pair) for pair in result.pairs]
    for pair in pairs:
        pair["other_role"] = LABELS[pair["other_role"]]
    return {
        "assessment_status": status,
        "termination_reason": record.termination_reason,
        "physics_hz": record.simulation.physics_hz,
        "end_tick": frames[-1].tick if frames else None,
        "collision": result.collision,
        "ego_collision": result.ego_collision,
        "first_collision_tick": result.first_collision_tick,
        "first_ego_collision_tick": result.first_ego_collision_tick,
        "collision_roles": [LABELS[role] for role in result.collision_roles],
        "pairs": pairs,
        "gap_warning": any(
            p["min_gap_m"] is not None and p["min_gap_m"] < warnings.gap_warning_m
            for p in pairs
        ),
        "ttc_warning": any(
            p["min_ttc_s"] is not None and p["min_ttc_s"] < warnings.ttc_warning_s
            for p in pairs
        ),
        "gap_warning_m": warnings.gap_warning_m,
        "ttc_warning_s": warnings.ttc_warning_s,
        "cut_in": {
            "prepare_start_time_s": record.prepare_start_time_s,
            "merge_start_time_s": record.merge_start_time_s,
            "merge_completion_time_s": record.merge_completion_time_s,
        },
        "summary_version": "web-longitudinal-v1",
    }
