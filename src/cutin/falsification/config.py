"""Search bounds and proposal seed, separate from scenario/backend seeds."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SearchConfig:
    budget: int = 20
    max_attempts: int = 100
    seed: int = 17


def validate_search_config(config: SearchConfig) -> None:
    for name in ("budget", "max_attempts", "seed"):
        value = getattr(config, name)
        minimum = 0 if name == "seed" else 1
        if type(value) is not int or value < minimum:
            raise ValueError(f"{name} must be an integer >= {minimum}")
