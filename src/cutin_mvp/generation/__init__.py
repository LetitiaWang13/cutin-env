"""Backend-independent numeric proposal generation."""

from cutin_mvp.generation.sampling import RejectedSample, sample
from cutin_mvp.generation.space import (
    ParameterRange,
    SearchSpace,
    validate_search_space,
)

__all__ = [
    "ParameterRange",
    "RejectedSample",
    "SearchSpace",
    "sample",
    "validate_search_space",
]
