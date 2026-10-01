"""Pure, offline collision/gap/TTC evaluation. Never reads a simulator object."""

from math import cos, isclose, isfinite

from cutin.core.geometry import half_extents
from cutin.core.log import SimulationLog, TerminationReason
from cutin.core.metrics import AssessmentStatus, Metrics, PairMetrics
from cutin.core.state import Role, VehicleState
from cutin.evaluation.config import EvaluationConfig, validate_evaluation_config


def observe_pair(
    ego: VehicleState, other: VehicleState
) -> tuple[float | None, float | None]:
    """Signed x bumper gap and positive TTC only when lateral bounds overlap.

    Either vehicle may trail. Heading-projected speed approximates x velocity;
    TTC does not predict lateral motion. Negative gap is not collision evidence.
    """
    ego_x, ego_y = half_extents(ego)
    other_x, other_y = half_extents(other)
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
        ttc = None  # An unrepresentable TTC is unavailable, never JSON infinity.
    return gap, ttc


def _validate_log(log: SimulationLog) -> int:
    """Input-boundary checks, not validation hooks on every internal record."""
    hz = log.simulator_settings.get("physics_hz")
    if type(hz) is not int or hz <= 0:
        raise ValueError("log needs positive integer physics_hz metadata")
    for index, entry in enumerate(log.entries):
        if type(entry.snapshot.tick) is not int or entry.snapshot.tick != index:
            raise ValueError(
                "log must include tick 0 and every consecutive physics tick"
            )
        for state in entry.snapshot.vehicles:
            values = (
                state.x_m,
                state.y_m,
                state.heading_rad,
                state.speed_mps,
                state.length_m,
                state.width_m,
            )
            try:
                finite = all(
                    not isinstance(value, bool)
                    and isinstance(value, (int, float))
                    and isfinite(value)
                    for value in values
                )
            except OverflowError:
                finite = False
            if (
                not finite
                or state.speed_mps < 0
                or state.length_m <= 0
                or state.width_m <= 0
            ):
                raise ValueError(
                    "log states need finite values, nonnegative speed, "
                    "and positive dimensions"
                )
            if not isinstance(state.role, Role) or type(state.collided) is not bool:
                raise ValueError(
                    "log states need Role identities and boolean collision flags"
                )
    if not isinstance(log.termination_reason, TerminationReason):
        raise ValueError("log needs a TerminationReason")
    if log.termination_reason == TerminationReason.HORIZON:
        if (
            not log.entries
            or log.entries[-1].snapshot.tick < 1
            or not isclose(
                log.entries[-1].snapshot.tick / hz,
                log.scenario.duration_s,
                rel_tol=0,
                abs_tol=1e-9,
            )
        ):
            raise ValueError("horizon log must reach its configured duration")
    elif log.termination_reason == TerminationReason.COLLISION:
        if not log.entries or not any(
            state.collided for state in log.entries[-1].snapshot.vehicles
        ):
            raise ValueError("collision termination needs terminal collision evidence")
    return hz


def _pair_minima(log: SimulationLog, role: Role) -> PairMetrics:
    gap_min = gap_tick = ttc_min = ttc_tick = None
    for entry in log.entries:
        snapshot = entry.snapshot
        for ego in snapshot.vehicles_for_role(Role.EGO):
            for other in snapshot.vehicles_for_role(role):
                gap, ttc = observe_pair(ego, other)
                if gap is not None and (gap_min is None or gap < gap_min):
                    gap_min, gap_tick = gap, snapshot.tick
                if ttc is not None and (ttc_min is None or ttc < ttc_min):
                    ttc_min, ttc_tick = ttc, snapshot.tick
    return PairMetrics(role, gap_min, gap_tick, ttc_min, ttc_tick)


def evaluate(log: SimulationLog, config: EvaluationConfig | None = None) -> Metrics:
    """Assess observed data, not controller safety or future lateral collisions.

    Complete means horizon reached or the target collision ended the run, not
    safe. Errors/road exits/cancellation and non-target collision runs cannot
    become eligible safety successes just because some observations are present.
    """
    config = EvaluationConfig() if config is None else config
    validate_evaluation_config(config)
    hz = _validate_log(log)
    first_collision = first_ego_collision = None
    collision_roles = ()
    for entry in log.entries:
        collided = {state.role for state in entry.snapshot.vehicles if state.collided}
        if collided and first_collision is None:
            first_collision = entry.snapshot.tick
            collision_roles = tuple(role for role in Role if role in collided)
        if Role.EGO in collided and first_ego_collision is None:
            first_ego_collision = entry.snapshot.tick
    reason = log.termination_reason
    if reason == TerminationReason.HORIZON:
        if first_collision is not None:
            raise ValueError(
                "horizon log contains a collision instead of collision termination"
            )
        status = AssessmentStatus.COMPLETE
    elif reason == TerminationReason.COLLISION:
        if first_collision != log.entries[-1].snapshot.tick:
            raise ValueError("collision run must stop on its first collision")
        status = (
            AssessmentStatus.COMPLETE
            if first_ego_collision is not None
            else AssessmentStatus.NON_TARGET_COLLISION
        )
    else:
        status = AssessmentStatus.INCOMPLETE
    # Missing target observations cannot establish a complete assessment,
    # even when the producer declares that the horizon was reached.
    if any(not entry.snapshot.vehicles_for_role(Role.EGO) for entry in log.entries):
        status = AssessmentStatus.INCOMPLETE
    pairs = tuple(_pair_minima(log, role) for role in (Role.FRONT, Role.CUT_IN))
    return Metrics(
        assessment_status=status,
        termination_reason=reason,
        physics_hz=hz,
        end_tick=log.entries[-1].snapshot.tick if log.entries else None,
        collision=first_collision is not None,
        ego_collision=first_ego_collision is not None,
        first_collision_tick=first_collision,
        first_ego_collision_tick=first_ego_collision,
        collision_roles=collision_roles,
        pairs=pairs,
        gap_warning=any(
            pair.min_gap_m is not None and pair.min_gap_m < config.gap_warning_m
            for pair in pairs
        ),
        ttc_warning=any(
            pair.min_ttc_s is not None and pair.min_ttc_s < config.ttc_warning_s
            for pair in pairs
        ),
        gap_warning_m=config.gap_warning_m,
        ttc_warning_s=config.ttc_warning_s,
    )
