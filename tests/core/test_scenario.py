"""Scenario parsing and boundary validation, independent of the input format."""

import json
import tomllib
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

import pytest

from cutin.core import (
    BaselinePolicy,
    CutInConfig,
    InitialConditions,
    InitialVehicle,
    RoadConfig,
    ScenarioConfig,
    VehicleDimensions,
    validate_scenario,
)


def test_baseline_round_trip() -> None:
    path = Path(__file__).resolve().parents[2] / "examples" / "baseline.toml"
    data = tomllib.loads(path.read_text())
    scenario = ScenarioConfig.from_dict(data)
    assert scenario == ScenarioConfig()
    assert scenario.to_dict() == data
    assert (
        ScenarioConfig.from_dict(json.loads(json.dumps(scenario.to_dict()))) == scenario
    )


def test_defaults_are_expanded_and_input_is_not_mutated() -> None:
    data = {"cut_in": {"start_time_s": 3.0}}
    scenario = ScenarioConfig.from_dict(data)
    assert data == {"cut_in": {"start_time_s": 3.0}}
    assert scenario.cut_in == CutInConfig(start_time_s=3.0)
    assert scenario.to_dict()["road"]["lane_count"] == 3
    output = scenario.to_dict()
    output["initial"]["ego"]["speed_mps"] = 0
    assert scenario.initial.ego.speed_mps == 25


def test_nested_records_are_immutable() -> None:
    scenario = ScenarioConfig()
    with pytest.raises(FrozenInstanceError):
        scenario.initial.ego.x_m = 100
    changed = replace(scenario, cut_in=replace(scenario.cut_in, start_time_s=4))
    assert scenario.cut_in.start_time_s == 2
    assert changed.cut_in.start_time_s == 4


@pytest.mark.parametrize(
    "data",
    [
        None,
        [],
        {"unknown": 1},
        {"schema_version": 2},
        {"schema_version": True},
        {"seed": True},
        {"seed": -1},
        {"seed": 1.5},
        {"duration_s": 0},
        {"duration_s": -1},
        {"duration_s": "10"},
        {"duration_s": True},
        {"duration_s": float("nan")},
        {"duration_s": float("inf")},
        {"duration_s": 10**1000},
        {"road": None},
        {"road": {"unexpected": 0}},
        {"road": {"lane_count": 2}},
        {"road": {"lane_count": True}},
        {"road": {"lane_width_m": 0}},
        {"road": {"lane_width_m": 1}},
        {"road": {"length_m": 100}},
        {"vehicles": {"length_m": 0}},
        {"vehicles": {"width_m": -1}},
        {"initial": []},
        {"initial": {"other": {}}},
        {"initial": {"ego": None}},
        {"initial": {"ego": {"x_m": 50}}},
        {"initial": {"ego": {"x_m": 50, "lane": 1, "speed_mps": 20, "other": 0}}},
        {"initial": {"ego": {"x_m": 50, "lane": 0, "speed_mps": 20}}},
        {"initial": {"ego": {"x_m": 50, "lane": True, "speed_mps": 20}}},
        {"initial": {"ego": {"x_m": 50, "lane": 1, "speed_mps": -1}}},
        {"initial": {"ego": {"x_m": 0, "lane": 1, "speed_mps": 20}}},
        {"initial": {"front": {"x_m": 40, "lane": 1, "speed_mps": 20}}},
        {"initial": {"front": {"x_m": 55, "lane": 1, "speed_mps": 20}}},
        {"cut_in": {"start_time_s": -1}},
        {"cut_in": {"target_speed_mps": -1}},
        {"cut_in": {"lane_change_duration_s": 0}},
        {"cut_in": {"preparation_timeout": 2}},
        {"baseline_policy": {"minimum_gap_m": 0}},
        {"baseline_policy": {"max_braking_mps2": 0}},
    ],
)
def test_invalid_external_inputs(data: object) -> None:
    with pytest.raises(ValueError):
        ScenarioConfig.from_dict(data)


def test_explicit_validation_checks_python_created_scenarios() -> None:
    with pytest.raises(ValueError, match="RoadConfig"):
        validate_scenario(ScenarioConfig(road={}))
    with pytest.raises(ValueError, match="InitialVehicle"):
        validate_scenario(ScenarioConfig(initial=InitialConditions(ego={})))


def test_construction_and_replace_do_not_run_validation() -> None:
    changed = replace(ScenarioConfig(), duration_s=-1)
    assert changed.duration_s == -1
    with pytest.raises(ValueError, match="duration_s"):
        validate_scenario(changed)


def test_toml_values_use_the_same_validation_boundary() -> None:
    data = tomllib.loads("[cut_in]\nlane_change_duration_s = 0.0\n")
    with pytest.raises(ValueError, match="lane_change_duration_s"):
        ScenarioConfig.from_dict(data)


def test_stationary_and_late_maneuver_are_valid() -> None:
    scenario = ScenarioConfig(
        initial=InitialConditions(ego=InitialVehicle(50, 1, 0)),
        cut_in=CutInConfig(start_time_s=20, target_speed_mps=0),
        baseline_policy=BaselinePolicy(desired_speed_mps=0),
    )
    validate_scenario(scenario)
    assert scenario.cut_in.start_time_s > scenario.duration_s


def test_same_x_in_separate_lanes_is_valid() -> None:
    scenario = ScenarioConfig(
        initial=InitialConditions(cut_in=InitialVehicle(50, 0, 25))
    )
    validate_scenario(scenario)
    assert scenario.initial.cut_in.x_m == scenario.initial.ego.x_m


def test_footprint_can_touch_road_boundary_but_not_another_vehicle() -> None:
    scenario = ScenarioConfig(initial=InitialConditions(ego=InitialVehicle(2.5, 1, 25)))
    validate_scenario(scenario)
    assert scenario.initial.ego.x_m == scenario.vehicles.length_m / 2
    touching = ScenarioConfig(
        vehicles=VehicleDimensions(width_m=4),
        road=RoadConfig(lane_width_m=4),
        initial=InitialConditions(cut_in=InitialVehicle(50, 0, 25)),
    )
    with pytest.raises(ValueError, match="overlap or touch"):
        validate_scenario(touching)
