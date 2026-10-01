"""Scenario records and validation at the input/execution boundary.

Defaults are illustrative. Constructing a record does not run validation;
from_dict() validates external input, and validate_scenario() checks scenarios
created or modified by Python callers before execution.
"""

from dataclasses import asdict, dataclass, field
from itertools import combinations
from typing import Any

from cutin.core._validation import integer, number, positive


@dataclass(frozen=True)
class RoadConfig:
    lane_count: int = 3
    lane_width_m: float = 4.0
    length_m: float = 2000.0


@dataclass(frozen=True)
class VehicleDimensions:
    length_m: float = 5.0
    width_m: float = 2.0


@dataclass(frozen=True)
class InitialVehicle:
    x_m: float
    lane: int
    speed_mps: float


@dataclass(frozen=True)
class InitialConditions:
    ego: InitialVehicle = field(default_factory=lambda: InitialVehicle(50.0, 1, 25.0))
    front: InitialVehicle = field(
        default_factory=lambda: InitialVehicle(110.0, 1, 25.0)
    )
    cut_in: InitialVehicle = field(
        default_factory=lambda: InitialVehicle(65.0, 0, 25.0)
    )


@dataclass(frozen=True)
class BaselinePolicy:
    desired_speed_mps: float = 25.0
    time_headway_s: float = 1.5
    minimum_gap_m: float = 2.0
    max_acceleration_mps2: float = 2.0
    max_braking_mps2: float = 6.0


@dataclass(frozen=True)
class CutInConfig:
    """Timed merge; start_time_s begins lateral motion, not preparation."""

    start_time_s: float = 2.0
    target_speed_mps: float = 25.0
    lane_change_duration_s: float = 2.0


@dataclass(frozen=True)
class ScenarioConfig:
    schema_version: int = 1
    seed: int = 42
    road: RoadConfig = field(default_factory=RoadConfig)
    duration_s: float = 10.0
    initial: InitialConditions = field(default_factory=InitialConditions)
    vehicles: VehicleDimensions = field(default_factory=VehicleDimensions)
    baseline_policy: BaselinePolicy = field(default_factory=BaselinePolicy)
    cut_in: CutInConfig = field(default_factory=CutInConfig)

    def to_dict(self) -> dict[str, Any]:
        """Expand defaults into plain values suitable for JSON output."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ScenarioConfig":
        """Parse TOML/JSON values, reject unknown fields, and validate once."""
        if not isinstance(data, dict):
            raise ValueError("scenario must be a mapping")
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
                        raise ValueError(f"{name} must be a mapping")
                    values[name] = constructor(**values[name])
            if "initial" in values:
                if not isinstance(values["initial"], dict):
                    raise ValueError("initial must be a mapping")
                initial = dict(values["initial"])
                for role in ("ego", "front", "cut_in"):
                    if role in initial:
                        if not isinstance(initial[role], dict):
                            raise ValueError(f"initial.{role} must be a mapping")
                        initial[role] = InitialVehicle(**initial[role])
                values["initial"] = InitialConditions(**initial)
            scenario = cls(**values)
        except TypeError as exc:
            raise ValueError(f"invalid scenario fields: {exc}") from exc
        validate_scenario(scenario)
        return scenario


def validate_scenario(scenario: ScenarioConfig) -> None:
    """Check input once; three-vehicle restrictions belong only to this MVP scene."""
    if not isinstance(scenario, ScenarioConfig):
        raise ValueError("scenario must be a ScenarioConfig")
    for name, expected in (
        ("road", RoadConfig),
        ("initial", InitialConditions),
        ("vehicles", VehicleDimensions),
        ("baseline_policy", BaselinePolicy),
        ("cut_in", CutInConfig),
    ):
        if not isinstance(getattr(scenario, name), expected):
            raise ValueError(f"{name} must be a {expected.__name__}")
    if type(scenario.schema_version) is not int or scenario.schema_version != 1:
        raise ValueError("unsupported scenario schema_version")
    integer("seed", scenario.seed)
    positive("duration_s", scenario.duration_s)
    road, dimensions = scenario.road, scenario.vehicles
    integer("lane_count", road.lane_count)
    if road.lane_count != 3:
        raise ValueError("lane_count must be 3 for the MVP")
    positive("lane_width_m", road.lane_width_m)
    positive("road length_m", road.length_m)
    positive("vehicle length_m", dimensions.length_m)
    positive("vehicle width_m", dimensions.width_m)
    if road.lane_width_m < dimensions.width_m:
        raise ValueError("vehicle width must fit within a lane")

    initial = scenario.initial
    states = (initial.ego, initial.front, initial.cut_in)
    for name, state in zip(("ego", "front", "cut_in"), states, strict=True):
        if not isinstance(state, InitialVehicle):
            raise ValueError(f"initial.{name} must be an InitialVehicle")
        number(f"initial.{name}.x_m", state.x_m)
        integer(f"initial.{name}.lane", state.lane)
        number(f"initial.{name}.speed_mps", state.speed_mps, minimum=0)
        half_length = dimensions.length_m / 2
        if not half_length <= state.x_m <= road.length_m - half_length:
            raise ValueError(
                f"initial.{name}.x_m={state.x_m} m is outside the road: "
                f"center must be within [{half_length}, "
                f"{road.length_m - half_length}] m to fit the vehicle footprint"
            )
    if (initial.ego.lane, initial.front.lane, initial.cut_in.lane) != (1, 1, 0):
        raise ValueError("ego/front must start in lane 1 and cut_in in lane 0")
    if initial.front.x_m <= initial.ego.x_m:
        raise ValueError(
            f"front must start ahead of ego: front x={initial.front.x_m} m, "
            f"ego x={initial.ego.x_m} m"
        )
    named_states = zip(("ego", "front", "cut_in"), states, strict=True)
    for (name_a, a), (name_b, b) in combinations(named_states, 2):
        if (
            abs(a.x_m - b.x_m) <= dimensions.length_m
            and abs(a.lane - b.lane) * road.lane_width_m <= dimensions.width_m
        ):
            raise ValueError(
                f"{name_a}/{name_b}: initial vehicle footprints overlap or touch; "
                f"center separation {abs(a.x_m - b.x_m):.3f} m must exceed "
                f"vehicle length {dimensions.length_m:.3f} m"
            )

    policy, cut_in = scenario.baseline_policy, scenario.cut_in
    number("desired_speed_mps", policy.desired_speed_mps, minimum=0)
    for name in (
        "time_headway_s",
        "minimum_gap_m",
        "max_acceleration_mps2",
        "max_braking_mps2",
    ):
        positive(name, getattr(policy, name))
    number("start_time_s", cut_in.start_time_s, minimum=0)
    number("target_speed_mps", cut_in.target_speed_mps, minimum=0)
    positive("lane_change_duration_s", cut_in.lane_change_duration_s)
