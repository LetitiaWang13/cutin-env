"""Minimal user workflow; component implementations and output handling are pending."""

from cutin.config import UserConfig
from cutin.core.record import ScenarioRecord
from cutin.generation.optim.optim_generator import OptimGenerator


def run(config: UserConfig) -> list[ScenarioRecord]:
    """Generate records; saving/playback will be connected at this outer boundary.

    Only the optimization-based generator is represented for now. Loading config
    files and selecting other generator types are not implemented.
    """
    generator = OptimGenerator(config)
    return generator.run()
