"""Observed lane-change duration."""

from dataclasses import dataclass
from math import isfinite
from typing import Literal

from cutin.core.record import ScenarioRecord


@dataclass(frozen=True)
class LaneChangeMeasurement:
    duration_s: float | None
    unavailable_reason: (
        Literal["merge_not_started", "merge_not_completed", "invalid_merge_timestamps"]
        | None
    ) = None


def observe(record: ScenarioRecord) -> LaneChangeMeasurement:
    """Measure the observed merge or explain why a duration is unavailable."""
    start = record.merge_start_time_s
    completion = record.merge_completion_time_s
    if start is None and completion is None:
        return LaneChangeMeasurement(None, "merge_not_started")
    if start is None or not isfinite(start) or start < 0:
        return LaneChangeMeasurement(None, "invalid_merge_timestamps")
    if completion is None:
        return LaneChangeMeasurement(None, "merge_not_completed")
    if not isfinite(completion) or completion <= start:
        return LaneChangeMeasurement(None, "invalid_merge_timestamps")
    return LaneChangeMeasurement(completion - start)


def compute(record: ScenarioRecord) -> float | None:
    """Return elapsed merge time in seconds when both timestamps are valid."""
    return observe(record).duration_s
