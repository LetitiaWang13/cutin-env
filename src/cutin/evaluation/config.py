"""Warning thresholds change interpretation, not simulation or raw minima."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class EvaluationConfig:
    gap_warning_m: float = 2.0
    ttc_warning_s: float = 1.5


def validate_evaluation_config(config: EvaluationConfig) -> None:
    for name in ("gap_warning_m", "ttc_warning_s"):
        value = getattr(config, name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(value)
            or value < 0
        ):
            raise ValueError(f"{name} must be finite and nonnegative")
