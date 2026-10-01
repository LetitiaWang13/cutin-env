"""Pure safety evaluation from normalized logs; no backend or policy imports."""

from cutin.evaluation.config import EvaluationConfig
from cutin.evaluation.safety import evaluate

__all__ = ["EvaluationConfig", "evaluate"]
