"""Bounded sequential search with injected simulation/evaluation operations."""

from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from random import Random

from cutin.core import (
    AssessmentStatus,
    Metrics,
    ScenarioConfig,
    SimulationLog,
    TerminationReason,
)
from cutin.core.scenario_config import validate_scenario
from cutin.falsification.config import SearchConfig, validate_search_config
from cutin.falsification.objective import eligible, rank
from cutin.generation import RejectedSample, SearchSpace, sample, validate_search_space


class SearchStopReason(StrEnum):
    BUDGET = "budget"
    ATTEMPT_CAP = "attempt_cap"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class SearchResult:
    base: ScenarioConfig
    space: SearchSpace
    config: SearchConfig
    best_log: SimulationLog | None
    best_metrics: Metrics | None
    best_attempt: int | None  # One-based proposal index, including rejected attempts.
    attempts: int
    completed: int
    ego_collisions: int
    rejected: int
    errors: int
    road_exits: int
    non_target_collisions: int
    cancelled: int
    incomplete: int
    stop_reason: SearchStopReason
    last_error: str | None = None
    last_rejection: str | None = None
    objective_version: str = "ego-collision-gap-ttc-v1"
    schema_version: int = 1

    @property
    def best_config(self) -> ScenarioConfig | None:
        return self.best_log.scenario if self.best_log is not None else None


def search(
    base: ScenarioConfig,
    space: SearchSpace,
    config: SearchConfig,
    simulate_fn: Callable[[ScenarioConfig], SimulationLog],
    evaluate_fn: Callable[[SimulationLog], Metrics],
) -> SearchResult:
    """Keep only the best full run. Exceptions count as errors, never successes.

    Cancellation stops the search instead of quietly launching another run.
    Ties preserve the earlier candidate; a found collision does not end the budget.
    """
    validate_scenario(base)
    validate_search_space(space)
    validate_search_config(config)
    rng = Random(config.seed)
    attempts = completed = ego_collisions = 0
    rejected = errors = road_exits = 0
    non_target = cancelled = incomplete = 0
    best_log = best_metrics = best_attempt = None
    stop = SearchStopReason.ATTEMPT_CAP
    last_error = last_rejection = None
    while attempts < config.max_attempts and completed < config.budget:
        attempts += 1
        try:
            candidate = sample(base, space, rng)
        except RejectedSample as error:
            last_rejection = str(error)
            rejected += 1
            continue
        try:
            log = simulate_fn(candidate)
            if log.scenario != candidate:
                raise ValueError("simulation result does not match its candidate")
            if log.termination_reason == TerminationReason.CANCELLED:
                cancelled += 1
                stop = SearchStopReason.CANCELLED
                break
            if log.termination_reason == TerminationReason.ERROR:
                last_error = log.diagnostic
                errors += 1
                continue
            if log.termination_reason == TerminationReason.ROAD_EXIT:
                road_exits += 1
                continue
            metrics = evaluate_fn(log)
            if metrics.termination_reason != log.termination_reason:
                raise ValueError(
                    "simulation/evaluation result does not match its input"
                )
        except Exception as error:
            # Backend/evaluator failures belong to the attempt cap. KeyboardInterrupt
            # and other BaseExceptions propagate rather than being swallowed.
            last_error = f"{type(error).__name__}: {error}"
            errors += 1
            continue
        if metrics.assessment_status == AssessmentStatus.NON_TARGET_COLLISION:
            non_target += 1
        elif not eligible(metrics):
            incomplete += 1
        else:
            completed += 1
            ego_collisions += int(metrics.ego_collision)
            if best_metrics is None or rank(metrics) < rank(best_metrics):
                best_log, best_metrics, best_attempt = log, metrics, attempts
    if completed == config.budget:
        stop = SearchStopReason.BUDGET
    return SearchResult(
        base=base,
        space=space,
        config=config,
        best_log=best_log,
        best_metrics=best_metrics,
        best_attempt=best_attempt,
        attempts=attempts,
        completed=completed,
        ego_collisions=ego_collisions,
        rejected=rejected,
        errors=errors,
        road_exits=road_exits,
        non_target_collisions=non_target,
        cancelled=cancelled,
        incomplete=incomplete,
        stop_reason=stop,
        last_error=last_error,
        last_rejection=last_rejection,
    )
