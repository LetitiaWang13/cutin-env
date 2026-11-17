"""Display summary of one ScenarioRecord: collision, gap and TTC per pair.

This is a Web-only convenience computed from the recorded frames. It is not the
team's Evaluator (cutin.evaluation), which is still being designed; swap it out
when that exists. Complete means the horizon was reached or a collision ended
the run. It never means the scenario is safe.
"""

from dataclasses import dataclass
from math import cos, isfinite, sin

from cutin.core.record import Frame, ScenarioRecord, VehicleState

ROLES = ("ego", "front", "cutin")
# Names shown in the page; the record's own field for the cut-in vehicle is `cutin`.
LABELS = {"ego": "ego", "front": "front", "cutin": "cut_in"}


@dataclass(frozen=True)
class Warnings:
    gap_warning_m: float
    ttc_warning_s: float


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
    """Signed x bumper gap and positive TTC, only while lateral bounds overlap.

    Either vehicle may trail. TTC uses heading-projected speed and does not
    predict lateral motion. A negative gap alone is not collision evidence.
    """
    ego_x, ego_y = _half_extents(ego)
    other_x, other_y = _half_extents(other)
    if abs(ego.y_m - other.y_m) > ego_y + other_y:
        return None, None
    dx = other.x_m - ego.x_m
    gap = abs(dx) - ego_x - other_x
    if not isfinite(gap):
        raise ValueError("pair gap is not representable as a finite number")
    closing = ego.speed_mps * cos(ego.heading_rad) - other.speed_mps * cos(
        other.heading_rad
    )
    if dx < 0:
        closing = -closing
    ttc = gap / closing if gap > 0 and closing > 0 else None
    if ttc is not None and (not isfinite(ttc) or ttc <= 0):
        ttc = None
    return gap, ttc


def _pair_minima(frames: list[Frame], role: str) -> dict:
    gap_min = gap_tick = ttc_min = ttc_tick = None
    for frame in frames:
        gap, ttc = observe_pair(frame.ego, getattr(frame, role))
        if gap is not None and (gap_min is None or gap < gap_min):
            gap_min, gap_tick = gap, frame.tick
        if ttc is not None and (ttc_min is None or ttc < ttc_min):
            ttc_min, ttc_tick = ttc, frame.tick
    return {
        "other_role": LABELS[role],
        "min_gap_m": gap_min,
        "min_gap_tick": gap_tick,
        "min_ttc_s": ttc_min,
        "min_ttc_tick": ttc_tick,
    }


def summarize(record: ScenarioRecord, warnings: Warnings) -> dict:
    frames = record.frames
    first_collision = first_ego_collision = None
    collision_roles: list[str] = []
    for frame in frames:
        collided = [role for role in ROLES if getattr(frame, role).collided]
        if collided and first_collision is None:
            first_collision = frame.tick
            collision_roles = [LABELS[role] for role in collided]
        if frame.ego.collided and first_ego_collision is None:
            first_ego_collision = frame.tick
    if record.termination_reason == "collision":
        status = (
            "complete" if first_ego_collision is not None else ("non_target_collision")
        )
    elif record.termination_reason == "horizon" and first_collision is None:
        status = "complete"
    else:
        status = "incomplete"
    pairs = [_pair_minima(frames, role) for role in ("front", "cutin")]
    return {
        "assessment_status": status,
        "termination_reason": record.termination_reason,
        "physics_hz": record.simulation.physics_hz,
        "end_tick": frames[-1].tick if frames else None,
        "collision": first_collision is not None,
        "ego_collision": first_ego_collision is not None,
        "first_collision_tick": first_collision,
        "first_ego_collision_tick": first_ego_collision,
        "collision_roles": collision_roles,
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
