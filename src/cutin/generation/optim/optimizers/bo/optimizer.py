"""BO/GP placeholder; no surrogate or acquisition function is implemented."""

from cutin.generation.optim.optimizers.interface import Optimizer


class BOOptimizer(Optimizer):
    """Will propose candidates using evaluated robustness observations."""
