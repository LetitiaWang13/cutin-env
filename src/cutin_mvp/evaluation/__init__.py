"""Pure safety evaluation from normalized logs; no backend or policy imports."""

from cutin_mvp.evaluation.config import EvaluationConfig
from cutin_mvp.evaluation.safety import evaluate

__all__ = ["EvaluationConfig", "evaluate"]
