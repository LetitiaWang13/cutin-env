"""Compose simulator and evaluator without owning either algorithm."""

from collections.abc import Callable
from dataclasses import dataclass

from cutin.core import Metrics, ScenarioConfig, SimulationLog
from cutin.evaluation import EvaluationConfig, evaluate
from cutin.evaluation.config import validate_evaluation_config
from cutin.falsification import SearchConfig, SearchResult, search
from cutin.generation import SearchSpace
from cutin.simulation import SimulatorConfig, simulate
from cutin.simulation.config import validate_simulator_config


@dataclass(frozen=True)
class RunResult:
    log: SimulationLog
    metrics: Metrics
    simulator_config: SimulatorConfig
    evaluation_config: EvaluationConfig


def run_scenario(
    scenario: ScenarioConfig,
    simulator_config: SimulatorConfig | None = None,
    evaluation_config: EvaluationConfig | None = None,
    *,
    simulate_fn: Callable[[ScenarioConfig, SimulatorConfig], SimulationLog] = simulate,
    evaluate_fn: Callable[[SimulationLog, EvaluationConfig], Metrics] = evaluate,
) -> RunResult:
    """Return an in-memory result; persistence and rendering are caller choices."""
    simulator_config = (
        SimulatorConfig() if simulator_config is None else simulator_config
    )
    evaluation_config = (
        EvaluationConfig() if evaluation_config is None else evaluation_config
    )
    # Do not spend a simulation run before detecting invalid warning settings.
    validate_evaluation_config(evaluation_config)
    log = simulate_fn(scenario, simulator_config)
    metrics = evaluate_fn(log, evaluation_config)
    return RunResult(log, metrics, simulator_config, evaluation_config)


@dataclass(frozen=True)
class SearchRunResult:
    search: SearchResult
    simulator_config: SimulatorConfig
    evaluation_config: EvaluationConfig


def search_scenarios(
    base: ScenarioConfig,
    space: SearchSpace,
    search_config: SearchConfig,
    simulator_config: SimulatorConfig | None = None,
    evaluation_config: EvaluationConfig | None = None,
    *,
    simulate_fn: Callable[[ScenarioConfig, SimulatorConfig], SimulationLog] = simulate,
    evaluate_fn: Callable[[SimulationLog, EvaluationConfig], Metrics] = evaluate,
) -> SearchRunResult:
    """Bind fixed settings; the falsification component owns the actual loop."""
    simulator_config = (
        SimulatorConfig() if simulator_config is None else simulator_config
    )
    evaluation_config = (
        EvaluationConfig() if evaluation_config is None else evaluation_config
    )
    validate_simulator_config(simulator_config)
    validate_evaluation_config(evaluation_config)
    result = search(
        base,
        space,
        search_config,
        lambda candidate: simulate_fn(candidate, simulator_config),
        lambda log: evaluate_fn(log, evaluation_config),
    )
    return SearchRunResult(result, simulator_config, evaluation_config)
