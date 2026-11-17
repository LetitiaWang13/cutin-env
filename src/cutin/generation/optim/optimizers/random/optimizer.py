"""Random-search placeholder; no sampling or stopping policy is implemented."""

from cutin.generation.optim.optimizers.interface import Optimizer


class RandomOptimizer(Optimizer):
    """Will propose independent candidates through the common optimizer interface."""
