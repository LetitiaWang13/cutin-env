"""Metric composition interface; no formulas or evaluation policies are fixed yet."""

from cutin.config import EvaluationConfig
from cutin.core.record import ScenarioRecord


class Evaluator:
    def __init__(self, config: EvaluationConfig) -> None:
        self.config = config

    def evaluate(self, record: ScenarioRecord):
        """Compute the requested metrics and robustness from the recorded episode."""
        raise NotImplementedError
