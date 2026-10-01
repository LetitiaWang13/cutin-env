"""Ego-focused eligibility and lexicographic ranking, not a composite score."""

from cutin.core import AssessmentStatus, Metrics, TerminationReason


def eligible(metrics: Metrics) -> bool:
    return metrics.assessment_status == AssessmentStatus.COMPLETE and (
        (
            metrics.termination_reason == TerminationReason.HORIZON
            and not metrics.collision
            and not metrics.ego_collision
        )
        or (
            metrics.termination_reason == TerminationReason.COLLISION
            and metrics.collision
            and metrics.ego_collision
        )
    )


def rank(metrics: Metrics) -> tuple[bool, float, float]:
    """Lower is better. Missing minima sort last; infinity is internal only."""
    if not eligible(metrics):
        raise ValueError("ineligible assessments cannot be ranked")
    gap = min(
        (pair.min_gap_m for pair in metrics.pairs if pair.min_gap_m is not None),
        default=float("inf"),
    )
    ttc = min(
        (pair.min_ttc_s for pair in metrics.pairs if pair.min_ttc_s is not None),
        default=float("inf"),
    )
    return not metrics.ego_collision, gap, ttc
