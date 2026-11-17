"""Payload parsing: strict fields, one clear message per rule."""

import copy

import pytest

from cutin.interfaces.web import inputs


def payload():
    return copy.deepcopy(inputs.defaults())


def test_defaults_round_trip_through_the_parsers():
    data = payload()
    scenario = inputs.parse_scenario(data["scenario"])
    assert scenario.cutin_controller.follow.desired_speed_mps == 27
    assert inputs.parse_simulation(data["simulation"]).physics_hz == 30
    assert inputs.parse_warnings(data["evaluation"]).gap_warning_m == 2.0


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda s: s["front"].update(x_m=55.05), "ego/front.*5.1"),
        (lambda s: s["front"].update(x_m=40), "ahead of ego"),
        (lambda s: s["ego"].update(x_m=1), "full vehicle"),
        (lambda s: s["ego"].update(lane=0), "lane 1"),
        (lambda s: s["ego"].update(speed_mps=-1), "at least 0"),
        (lambda s: s.update(duration_s=61), "limited to 60"),
        (lambda s: s.update(seed=1.5), "whole number"),
        (lambda s: s.update(seed=True), "must be a number"),
        (lambda s: s.update(extra=1), "unknown fields"),
        (lambda s: s["road"].update(lane_width_m=1), "fit within a lane"),
        (lambda s: s["ego_idm"].update(time_headway_s=0), "positive"),
        (lambda s: s["cutin_controller"].update(warmup_s=-1), "at least 0"),
        (lambda s: s["cutin_controller"].update(target_gap_m=float("nan")), "finite"),
    ],
)
def test_scenario_rejections(mutate, message):
    data = payload()["scenario"]
    mutate(data)
    with pytest.raises(ValueError, match=message):
        inputs.parse_scenario(data)


def test_adjacent_lanes_may_share_x_and_decimal_boundary_is_accepted():
    data = payload()["scenario"]
    data["cutin"]["x_m"] = data["ego"]["x_m"]
    inputs.parse_scenario(data)
    data["front"]["x_m"] = data["ego"]["x_m"] + 5.1
    inputs.parse_scenario(data)


@pytest.mark.parametrize(
    "change",
    [
        {"backend": "other"},
        {"physics_hz": 121},
        {"physics_hz": 30, "controller_hz": 7},
        {"controller_hz": 0},
    ],
)
def test_simulation_rejections(change):
    data = payload()["simulation"]
    data.update(change)
    with pytest.raises(ValueError):
        inputs.parse_simulation(data)
