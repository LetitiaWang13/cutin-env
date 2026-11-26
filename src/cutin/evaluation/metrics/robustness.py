"""Signed robustness of the TTC and LCT upper-limit targets."""

from dataclasses import dataclass

from cutin.config import EvaluationConfig, RobustnessTargets
from cutin.core.record import ScenarioRecord
from cutin.evaluation.metrics import lct, ttc


@dataclass(frozen=True)
class RobustnessResult:
    """Target score and measured-minus-target margins, in seconds."""

    score_s: float | None
    ttc_margin_s: float | None
    lct_margin_s: float | None
    unavailable_reasons: tuple[str, ...] = ()


def from_measurements(
    cutin_min_ttc_s: float | None,
    lane_change: lct.LaneChangeMeasurement,
    targets: RobustnessTargets,
) -> RobustnessResult:
    """Combine margins using max. A negative score meets both strict limits."""
    ttc_margin = None
    lct_margin = None
    unavailable = []
    if cutin_min_ttc_s is None:
        unavailable.append("no_valid_cutin_ttc")
    else:
        ttc_margin = cutin_min_ttc_s - targets.ttc_upper_s
    if lane_change.duration_s is None:
        unavailable.append(lane_change.unavailable_reason or "lct_unavailable")
    else:
        lct_margin = lane_change.duration_s - targets.lct_upper_s
    score = (
        max(ttc_margin, lct_margin)
        if ttc_margin is not None and lct_margin is not None
        else None
    )
    return RobustnessResult(score, ttc_margin, lct_margin, tuple(unavailable))


def compute(
    record: ScenarioRecord, config: EvaluationConfig
) -> RobustnessResult | None:
    """Score a record directly when robustness targets are configured."""
    targets = config.robustness_targets
    if targets is None:
        return None
    _, cutin = ttc.compute(record)
    return from_measurements(cutin.min_ttc_s, lct.observe(record), targets)
