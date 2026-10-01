"""Compose simulator and evaluator without owning either algorithm."""

from collections.abc import Callable
from dataclasses import dataclass

from cutin.core import Metrics, ScenarioConfig, SimulationLog
from cutin.evaluation import EvaluationConfig, evaluate
from cutin.evaluation.config import validate_evaluation_config
from cutin.simulation import SimulatorConfig, simulate


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
