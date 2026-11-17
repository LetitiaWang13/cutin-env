"""One-scenario execution interface, independent of backend libraries."""

from cutin.config import SimulationConfig
from cutin.core.record import ScenarioRecord
from cutin.core.scenario import ScenarioConfig


class Simulator:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def run(self, scenario: ScenarioConfig) -> ScenarioRecord:
        raise NotImplementedError
