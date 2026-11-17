"""Parse and validate the Web form payload into the framework's config objects.

The payload mirrors ScenarioConfig/SimulationConfig field names, so a saved run's
record.json can be read back with the same names. Unknown fields are rejected.
Limits here are Web-only conveniences; they do not change the adapter.
"""

from dataclasses import asdict, dataclass, fields
from itertools import combinations
from math import isfinite
from typing import Any

from cutin.config import CutInConfig, IDMConfig, SimulationConfig
from cutin.core.scenario import InitialVehicleState, RoadConfig, ScenarioConfig

# highway-env's default vehicle footprint; the adapter copies it into every frame.
VEHICLE_LENGTH_M = 5.0
VEHICLE_WIDTH_M = 2.0
POSITION_CLEARANCE_M = 0.1
LANES = {"ego": 1, "front": 1, "cutin": 0}  # The fixed three-lane scene.
MAX_DURATION_S = 60
MAX_PHYSICS_HZ = 120


@dataclass(frozen=True)
class WarningConfig:
    """Interpretation thresholds only; they never change the simulation."""

    gap_warning_m: float = 2.0
    ttc_warning_s: float = 1.5


def default_scenario() -> ScenarioConfig:
    """Team example scenario, with the front car far enough ahead for the merge."""
    return ScenarioConfig(
        road=RoadConfig(length_m=2000, lane_width_m=4, speed_limit_mps=40),
        duration_s=12,
        seed=17,
        ego=InitialVehicleState(x_m=50, lane=1, speed_mps=25),
        front=InitialVehicleState(x_m=170, lane=1, speed_mps=25),
        cutin=InitialVehicleState(x_m=90, lane=0, speed_mps=23),
        ego_idm=IDMConfig(desired_speed_mps=25),
        front_idm=IDMConfig(desired_speed_mps=25),
        cutin_controller=CutInConfig(
            target_gap_m=25,
            lane_change_duration_s=2,
            follow=IDMConfig(desired_speed_mps=27),
        ),
    )


def default_simulation() -> SimulationConfig:
    return SimulationConfig(backend="highway-env", physics_hz=30, controller_hz=10)


def defaults() -> dict[str, Any]:
    return {
        "scenario": asdict(default_scenario()),
        "simulation": asdict(default_simulation()),
        "evaluation": asdict(WarningConfig()),
        "vehicle": {"length_m": VEHICLE_LENGTH_M, "width_m": VEHICLE_WIDTH_M},
    }


def _number(name: str, value: Any, *, minimum=None, positive=False, integer=False):
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a number")
    if integer and not isinstance(value, int):
        if not (isinstance(value, float) and value.is_integer()):
            raise ValueError(f"{name} must be a whole number")
        value = int(value)
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    if minimum is not None and value < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return value


def _build(cls, data: Any, name: str, nested: dict[str, Any] | None = None):
    """Construct a dataclass from a mapping: known fields only, no silent defaults."""
    if not isinstance(data, dict):
        raise ValueError(f"{name} must be a mapping")
    allowed = {field.name for field in fields(cls)}
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(f"{name} has unknown fields: {', '.join(sorted(unknown))}")
    values = dict(data)
    for key, constructor in (nested or {}).items():
        if key in values:
            values[key] = constructor(values[key], f"{name}.{key}")
    try:
        return cls(**values)
    except TypeError as error:
        raise ValueError(f"invalid {name}: {error}") from error


def _idm(data: Any, name: str) -> IDMConfig:
    idm = _build(IDMConfig, data, name)
    _number(f"{name}.desired_speed_mps", idm.desired_speed_mps, minimum=0)
    for field in fields(IDMConfig):
        if field.name != "desired_speed_mps":
            _number(f"{name}.{field.name}", getattr(idm, field.name), positive=True)
    return idm


def _cutin(data: Any, name: str) -> CutInConfig:
    config = _build(CutInConfig, data, name, {"follow": _idm})
    _number(f"{name}.target_gap_m", config.target_gap_m)
    _number(
        f"{name}.lane_change_duration_s", config.lane_change_duration_s, positive=True
    )
    _number(f"{name}.warmup_s", config.warmup_s, minimum=0)
    for field in (
        "trigger_tolerance_m",
        "completion_position_tolerance_m",
        "completion_heading_tolerance_rad",
    ):
        _number(f"{name}.{field}", getattr(config, field), positive=True)
    return config


def parse_scenario(data: Any) -> ScenarioConfig:
    scenario = _build(
        ScenarioConfig,
        data,
        "scenario",
        {
            "road": lambda value, name: _build(RoadConfig, value, name),
            "ego": lambda value, name: _build(InitialVehicleState, value, name),
            "front": lambda value, name: _build(InitialVehicleState, value, name),
            "cutin": lambda value, name: _build(InitialVehicleState, value, name),
            "ego_idm": _idm,
            "front_idm": _idm,
            "cutin_controller": _cutin,
        },
    )
    _number("seed", scenario.seed, integer=True)
    _number("duration_s", scenario.duration_s, positive=True)
    if scenario.duration_s > MAX_DURATION_S:
        raise ValueError(f"web runs are limited to {MAX_DURATION_S} seconds")
    road = scenario.road
    _number("road.length_m", road.length_m, positive=True)
    _number("road.lane_width_m", road.lane_width_m, positive=True)
    _number("road.speed_limit_mps", road.speed_limit_mps, positive=True)
    if road.lane_width_m < VEHICLE_WIDTH_M:
        raise ValueError("vehicle width must fit within a lane")
    states = {"ego": scenario.ego, "front": scenario.front, "cutin": scenario.cutin}
    for role, state in states.items():
        _number(f"{role}.x_m", state.x_m)
        _number(f"{role}.speed_mps", state.speed_mps, minimum=0)
        if state.lane != LANES[role]:
            raise ValueError(
                f"{role} must start in lane {LANES[role]} for this three-lane scene"
            )
        half = VEHICLE_LENGTH_M / 2
        if not half <= state.x_m <= road.length_m - half:
            raise ValueError(
                f"{role}: center x {state.x_m:.3f} m must be within "
                f"{half:.3f}-{road.length_m - half:.3f} m so the full vehicle "
                "stays on the road."
            )
    if scenario.front.x_m <= scenario.ego.x_m:
        raise ValueError(
            f"Front must be ahead of ego: front x {scenario.front.x_m:.3f} m, "
            f"ego x {scenario.ego.x_m:.3f} m."
        )
    required = VEHICLE_LENGTH_M + POSITION_CLEARANCE_M
    for (name_a, a), (name_b, b) in combinations(states.items(), 2):
        lateral = abs(a.lane - b.lane) * road.lane_width_m
        separation = abs(a.x_m - b.x_m)
        if lateral <= VEHICLE_WIDTH_M and separation + 1e-9 < required:
            raise ValueError(
                f"{name_a}/{name_b}: center separation {separation:.9f} m; "
                f"require at least {required:.3f} m "
                f"(vehicle length {VEHICLE_LENGTH_M:.3f} m "
                f"+ clearance {POSITION_CLEARANCE_M:.3f} m). "
                "Move these vehicles farther apart."
            )
    return scenario


def parse_simulation(data: Any) -> SimulationConfig:
    simulation = _build(SimulationConfig, data, "simulation")
    if simulation.backend != "highway-env":
        raise ValueError('simulation.backend must be "highway-env"')
    physics = _number("physics_hz", simulation.physics_hz, positive=True, integer=True)
    controller = _number(
        "controller_hz", simulation.controller_hz, positive=True, integer=True
    )
    if physics > MAX_PHYSICS_HZ:
        raise ValueError(f"web runs are limited to {MAX_PHYSICS_HZ} physics Hz")
    if physics % controller:
        raise ValueError("physics_hz must be an integer multiple of controller_hz")
    return SimulationConfig(simulation.backend, physics, controller)


def parse_warnings(data: Any) -> WarningConfig:
    config = _build(WarningConfig, data, "evaluation")
    for field in fields(WarningConfig):
        _number(f"evaluation.{field.name}", getattr(config, field.name), minimum=0)
    return config
