"""Longitudinal bumper gap and time to collision for ego vehicle pairs."""

from collections import Counter
from dataclasses import dataclass, field
from math import cos, isfinite, sin

from cutin.core.record import Frame, ScenarioRecord, VehicleState


@dataclass(frozen=True)
class TTCUnavailableCounts:
    """Frame counts by the first reason a positive TTC was unavailable."""

    no_lateral_overlap: int = 0
    nonpositive_gap: int = 0
    not_closing: int = 0
    invalid_ttc: int = 0


@dataclass(frozen=True)
class PairMinima:
    """Minimum gap and positive TTC across observed frames for one ego pair."""

    other_role: str
    min_gap_m: float | None
    min_gap_tick: int | None
    min_ttc_s: float | None
    min_ttc_tick: int | None
    ttc_unavailable_counts: TTCUnavailableCounts = field(
        default_factory=TTCUnavailableCounts
    )


def _half_extents(vehicle: VehicleState) -> tuple[float, float]:
    """Conservative world-axis half extents of a heading-rotated rectangle."""
    c, s = abs(cos(vehicle.heading_rad)), abs(sin(vehicle.heading_rad))
    return (
        (vehicle.length_m * c + vehicle.width_m * s) / 2,
        (vehicle.length_m * s + vehicle.width_m * c) / 2,
    )


def observe_pair(
    ego: VehicleState, other: VehicleState
) -> tuple[float | None, float | None]:
    """Return signed x bumper gap and positive TTC while lateral bounds overlap."""
    gap, ttc, _ = observe_pair_with_reason(ego, other)
    return gap, ttc


def observe_pair_with_reason(
    ego: VehicleState, other: VehicleState
) -> tuple[float | None, float | None, str | None]:
    """Return bumper gap, positive TTC, and the reason if TTC is unavailable.

    Either vehicle may trail. TTC assumes constant longitudinal velocity. It
    uses heading-projected speed and does not predict lateral motion. A negative
    gap alone is not collision evidence.
    """
    ego_x, ego_y = _half_extents(ego)
    other_x, other_y = _half_extents(other)
    if abs(ego.y_m - other.y_m) > ego_y + other_y:
        return None, None, "no_lateral_overlap"
    dx = other.x_m - ego.x_m
    gap = abs(dx) - ego_x - other_x
    if not isfinite(gap):
        raise ValueError("pair gap is not representable as a finite number")
    if gap <= 0:
        return gap, None, "nonpositive_gap"
    closing = ego.speed_mps * cos(ego.heading_rad) - other.speed_mps * cos(
        other.heading_rad
    )
    if dx < 0:
        closing = -closing
    if not isfinite(closing):
        return gap, None, "invalid_ttc"
    if closing <= 0:
        return gap, None, "not_closing"
    ttc = gap / closing
    if not isfinite(ttc) or ttc <= 0:
        return gap, None, "invalid_ttc"
    return gap, ttc, None


def _pair_minima(frames: list[Frame], other_role: str) -> PairMinima:
    gap_min = gap_tick = ttc_min = ttc_tick = None
    unavailable_counts: Counter[str] = Counter()
    for frame in frames:
        gap, ttc, reason = observe_pair_with_reason(
            frame.ego, getattr(frame, other_role)
        )
        if reason is not None:
            unavailable_counts[reason] += 1
        if gap is not None and (gap_min is None or gap < gap_min):
            gap_min, gap_tick = gap, frame.tick
        if ttc is not None and (ttc_min is None or ttc < ttc_min):
            ttc_min, ttc_tick = ttc, frame.tick
    return PairMinima(
        other_role,
        gap_min,
        gap_tick,
        ttc_min,
        ttc_tick,
        TTCUnavailableCounts(**unavailable_counts),
    )


def compute(record: ScenarioRecord) -> tuple[PairMinima, PairMinima]:
    """Measure ego–front and ego–cut-in over the full recorded episode.

    Unavailable minima and their ticks are None. Ties retain the first sample.
    """
    return _pair_minima(record.frames, "front"), _pair_minima(record.frames, "cutin")
