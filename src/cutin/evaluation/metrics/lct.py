"""Observed lane-change duration."""

from cutin.core.record import ScenarioRecord


def compute(record: ScenarioRecord) -> float | None:
    """Return elapsed merge time in seconds, or None for an incomplete merge."""
    start = record.merge_start_time_s
    completion = record.merge_completion_time_s
    if start is None or completion is None:
        return None
    return completion - start
