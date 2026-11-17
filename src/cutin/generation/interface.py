"""Common entry point for generator types."""

from cutin.config import UserConfig
from cutin.core.record import ScenarioRecord


class Generator:
    def __init__(self, config: UserConfig) -> None:
        self.config = config

    def run(self) -> list[ScenarioRecord]:
        raise NotImplementedError
