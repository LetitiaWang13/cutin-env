"""Optimization-based generator; no search loop is implemented yet."""

from cutin.core.record import ScenarioRecord
from cutin.generation.interface import Generator


class OptimGenerator(Generator):
    def run(self) -> list[ScenarioRecord]:
        """Resolve targets/space, assemble components, and own the optimization loop.

        This is where optimizer proposals will be simulated, evaluated, and fed
        back. Optimizer implementations must not duplicate that execution loop.
        """
        raise NotImplementedError
