"""Optimization method seam; candidate and feedback formats are pending."""


class Optimizer:
    def suggest(self):
        """Propose candidate parameters, not execute a simulation."""
        raise NotImplementedError

    def update(self, parameters, metrics) -> None:
        """Receive feedback from the generator-owned loop."""
        raise NotImplementedError
