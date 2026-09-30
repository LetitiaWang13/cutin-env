"""Concrete, simulator-independent scenarios and explicit JSON validation.

Defaults are illustrative. A scenario contains no search ranges or backend
settings. Initial positions are vehicle centres; all values use SI units.
"""

from dataclasses import asdict, dataclass, field
from itertools import combinations
from typing import Any

from cutin.core._validation import integer, number, positive


@dataclass(frozen=True, slots=True)
class RoadConfig:
    lane_count: int = 3
    lane_width_m: float = 4.0
    length_m: float = 2000.0

    def __post_init__(self) -> None:
        if type(self.lane_count) is not int or self.lane_count != 3:
            raise ValueError("lane_count must be 3 for the MVP")
        positive("lane_width_m", self.lane_width_m)
        positive("length_m", self.length_m)


@dataclass(frozen=True, slots=True)
class VehicleDimensions:
    length_m: float = 5.0
    width_m: float = 2.0

    def __post_init__(self) -> None:
        positive("vehicle length_m", self.length_m)
        positive("vehicle width_m", self.width_m)


@dataclass(frozen=True, slots=True)
class InitialVehicle:
    x_m: float
    lane: int
    speed_mps: float

    def __post_init__(self) -> None:
        number("x_m", self.x_m)
        integer("lane", self.lane)
        if self.lane not in (0, 1, 2):
            raise ValueError("lane must be 0, 1, or 2")
        number("speed_mps", self.speed_mps, minimum=0)


@dataclass(frozen=True, slots=True)
class InitialConditions:
    ego: InitialVehicle = field(default_factory=lambda: InitialVehicle(50.0, 1, 25.0))
    front: InitialVehicle = field(
        default_factory=lambda: InitialVehicle(110.0, 1, 25.0)
    )
    cut_in: InitialVehicle = field(
        default_factory=lambda: InitialVehicle(65.0, 0, 25.0)
    )

    def __post_init__(self) -> None:
        for name in ("ego", "front", "cut_in"):
            if not isinstance(getattr(self, name), InitialVehicle):
                raise ValueError(f"initial.{name} must be an InitialVehicle")
        if (self.ego.lane, self.front.lane, self.cut_in.lane) != (1, 1, 0):
            raise ValueError("ego/front must start in lane 1 and cut_in in lane 0")
        if self.front.x_m <= self.ego.x_m:
            raise ValueError("front must start ahead of ego")


@dataclass(frozen=True, slots=True)
class BaselinePolicy:
    """Fixed car-following targets; implementation is added in the control step."""

    desired_speed_mps: float = 25.0
    time_headway_s: float = 1.5
    minimum_gap_m: float = 2.0
    max_acceleration_mps2: float = 2.0
    max_braking_mps2: float = 6.0

    def __post_init__(self) -> None:
        number("desired_speed_mps", self.desired_speed_mps, minimum=0)
        for name in (
            "time_headway_s",
            "minimum_gap_m",
            "max_acceleration_mps2",
            "max_braking_mps2",
        ):
            positive(name, getattr(self, name))


@dataclass(frozen=True, slots=True)
class CutInConfig:
    """Timed lane-0-to-lane-1 reference; no relative-position preparation."""

    start_time_s: float = 2.0
    target_speed_mps: float = 25.0
    lane_change_duration_s: float = 2.0

    def __post_init__(self) -> None:
        number("start_time_s", self.start_time_s, minimum=0)
        number("target_speed_mps", self.target_speed_mps, minimum=0)
        positive("lane_change_duration_s", self.lane_change_duration_s)


@dataclass(frozen=True, slots=True)
class ScenarioConfig:
    schema_version: int = 1
    seed: int = 42
    road: RoadConfig = field(default_factory=RoadConfig)
    duration_s: float = 10.0
    initial: InitialConditions = field(default_factory=InitialConditions)
    vehicles: VehicleDimensions = field(default_factory=VehicleDimensions)
    baseline_policy: BaselinePolicy = field(default_factory=BaselinePolicy)
    cut_in: CutInConfig = field(default_factory=CutInConfig)

    def __post_init__(self) -> None:
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("unsupported scenario schema_version")
        integer("seed", self.seed)
        positive("duration_s", self.duration_s)
        for name, expected in (
            ("road", RoadConfig),
            ("initial", InitialConditions),
            ("vehicles", VehicleDimensions),
            ("baseline_policy", BaselinePolicy),
            ("cut_in", CutInConfig),
        ):
            if not isinstance(getattr(self, name), expected):
                raise ValueError(f"{name} must be a {expected.__name__}")

        if self.road.lane_width_m < self.vehicles.width_m:
            raise ValueError("vehicle width must fit within a lane")
        states = (self.initial.ego, self.initial.front, self.initial.cut_in)
        half_length = self.vehicles.length_m / 2
        for state in states:
            if not half_length <= state.x_m <= self.road.length_m - half_length:
                raise ValueError("initial vehicle footprint must be inside the road")
        for a, b in combinations(states, 2):
            if (
                abs(a.x_m - b.x_m) <= self.vehicles.length_m
                and abs(a.lane - b.lane) * self.road.lane_width_m
                <= self.vehicles.width_m
            ):
                raise ValueError("initial vehicle footprints overlap or touch")

    def to_dict(self) -> dict[str, Any]:
        """Return plain JSON values with every default expanded."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ScenarioConfig":
        """Validate a JSON object; reject unknown fields at every level."""
        if not isinstance(data, dict):
            raise ValueError("scenario must be a JSON object")
        values = dict(data)
        constructors = {
            "road": RoadConfig,
            "vehicles": VehicleDimensions,
            "baseline_policy": BaselinePolicy,
            "cut_in": CutInConfig,
        }
        try:
            for name, constructor in constructors.items():
                if name in values:
                    if not isinstance(values[name], dict):
                        raise ValueError(f"{name} must be a JSON object")
                    values[name] = constructor(**values[name])
            if "initial" in values:
                if not isinstance(values["initial"], dict):
                    raise ValueError("initial must be a JSON object")
                initial = dict(values["initial"])
                for role in ("ego", "front", "cut_in"):
                    if role in initial:
                        if not isinstance(initial[role], dict):
                            raise ValueError(f"initial.{role} must be a JSON object")
                        initial[role] = InitialVehicle(**initial[role])
                values["initial"] = InitialConditions(**initial)
            return cls(**values)
        except TypeError as exc:
            raise ValueError(f"invalid scenario fields: {exc}") from exc
