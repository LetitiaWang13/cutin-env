"""Backend-independent numeric proposal generation."""

from cutin.generation.sampling import RejectedSample, sample
from cutin.generation.space import ParameterRange, SearchSpace, validate_search_space

__all__ = [
    "ParameterRange",
    "RejectedSample",
    "SearchSpace",
    "sample",
    "validate_search_space",
]
