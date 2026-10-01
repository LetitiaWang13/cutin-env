"""One independent uniform proposal per call, using the caller's local RNG."""

from random import Random

from cutin.core import ScenarioConfig
from cutin.core.scenario_config import validate_scenario
from cutin.generation.space import SearchSpace, validate_search_space


class RejectedSample(ValueError):
    """A draw violated scenario constraints; count the attempt, do not retry here."""


def sample(base: ScenarioConfig, space: SearchSpace, rng: Random) -> ScenarioConfig:
    """Return one valid concrete scenario or raise RejectedSample.

    Configuration errors raise ordinary ValueError before consuming RNG state.
    Fixed ranges consume no random draw. Alphabetical path order makes input
    insertion order irrelevant. Seeds, lanes, road, and baseline policy stay fixed.
    Backend-specific limits still belong to the simulator, not generation.
    """
    validate_scenario(base)
    validate_search_space(space)
    values = base.to_dict()
    for item in sorted(space.ranges, key=lambda item: item.path):
        target = values
        parts = item.path.split(".")
        for part in parts[:-1]:
            target = target[part]
        target[parts[-1]] = (
            item.low if item.low == item.high else rng.uniform(item.low, item.high)
        )
    try:
        return ScenarioConfig.from_dict(values)
    except ValueError as error:
        raise RejectedSample(str(error)) from error
