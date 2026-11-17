"""Explicit numeric proposal ranges, separate from concrete scenario values."""

from dataclasses import dataclass
from math import isfinite
from typing import Any

ALLOWED_PATHS = (
    "initial.ego.x_m",
    "initial.ego.speed_mps",
    "initial.front.x_m",
    "initial.front.speed_mps",
    "initial.cut_in.x_m",
    "initial.cut_in.speed_mps",
    "cut_in.start_time_s",
    "cut_in.target_speed_mps",
    "cut_in.lane_change_duration_s",
)


@dataclass(frozen=True)
class ParameterRange:
    path: str
    low: float
    high: float


@dataclass(frozen=True)
class SearchSpace:
    ranges: tuple[ParameterRange, ...] = ()
    schema_version: int = 1

    def to_dict(self) -> dict[str, Any]:
        validate_search_space(self)
        return {
            "schema_version": self.schema_version,
            "ranges": {item.path: [item.low, item.high] for item in self.ranges},
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SearchSpace":
        if not isinstance(data, dict):
            raise ValueError("search space must be an object")
        if set(data) - {"schema_version", "ranges"}:
            raise ValueError("unknown search-space fields")
        ranges = data.get("ranges", {})
        if not isinstance(ranges, dict):
            raise ValueError("ranges must be an object mapping paths to [low, high]")
        items = []
        for path, bounds in ranges.items():
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                raise ValueError(f"{path} needs [low, high]")
            items.append(ParameterRange(path, bounds[0], bounds[1]))
        space = cls(tuple(items), data.get("schema_version", 1))
        validate_search_space(space)
        return space


def validate_search_space(space: SearchSpace) -> None:
    """Check proposal syntax here; scenario constraints are checked after a draw."""
    if not isinstance(space, SearchSpace):
        raise ValueError("space must be a SearchSpace")
    if type(space.schema_version) is not int or space.schema_version != 1:
        raise ValueError("unsupported search-space schema_version")
    if not isinstance(space.ranges, tuple):
        raise ValueError("ranges must be a tuple of ParameterRange records")
    seen = set()
    for item in space.ranges:
        if not isinstance(item, ParameterRange):
            raise ValueError("ranges must contain ParameterRange records")
        if not isinstance(item.path, str) or item.path not in ALLOWED_PATHS:
            raise ValueError(f"unsupported search path: {item.path}")
        if item.path in seen:
            raise ValueError(f"duplicate search path: {item.path}")
        seen.add(item.path)
        try:
            valid = (
                all(
                    not isinstance(value, bool)
                    and isinstance(value, (int, float))
                    and isfinite(value)
                    for value in (item.low, item.high)
                )
                and item.low <= item.high
                and isfinite(item.high - item.low)
            )
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(
                f"{item.path} needs finite ordered bounds and a finite span"
            )
