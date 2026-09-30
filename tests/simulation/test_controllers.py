"""Driving computations use synthetic snapshots, not backend objects."""

from dataclasses import replace
from math import pi

import pytest

from cutin.core import CutInConfig, Role, ScenarioConfig, StateSnapshot, VehicleState
from cutin.core.scenario_config import BaselinePolicy
from cutin.simulation import SimulatorConfig
from cutin.simulation.controllers import (
    DrivingPolicy,
    following_acceleration,
    merge_reference,
    track_lateral,
)


def state(tick=0, *, cut_y=0, cut_speed=25, cut_heading=0, collided=False):
    return StateSnapshot(
        tick,
        (
            VehicleState(Role.EGO, 50, 4, 0, 25, 5, 2),
            VehicleState(Role.FRONT, 110, 4, 0, 25, 5, 2),
            VehicleState(
                Role.CUT_IN, 65, cut_y, cut_heading, cut_speed, 5, 2, collided
            ),
        ),
    )


def test_free_road_acceleration_and_stop_target() -> None:
    ego = replace(state().vehicles[0], speed_mps=0)
    snapshot = StateSnapshot(0, (ego,))
    assert following_acceleration(ego, snapshot, BaselinePolicy()) == 2
    assert (
        following_acceleration(ego, snapshot, BaselinePolicy(desired_speed_mps=0)) == 0
    )
    moving = replace(ego, speed_mps=25)
    assert (
        following_acceleration(
            moving, StateSnapshot(0, (moving,)), BaselinePolicy(desired_speed_mps=0)
        )
        == -6
    )


def test_partial_intrusion_triggers_braking_before_lane_change_completion() -> None:
    separate = state()
    ego = separate.vehicles[0]
    clear = following_acceleration(ego, separate, BaselinePolicy())
    intruding = replace(separate.vehicles[2], x_m=60, y_m=2.1)
    snapshot = replace(separate, vehicles=(ego, separate.vehicles[1], intruding))
    assert following_acceleration(ego, snapshot, BaselinePolicy()) == -6
    assert clear > -6


def test_leaders_can_share_a_role() -> None:
    follower = VehicleState(Role.FRONT, 50, 4, 0, 25, 5, 2)
    leader = replace(follower, x_m=60, speed_mps=0)
    assert (
        following_acceleration(
            follower, StateSnapshot(0, (follower, leader)), BaselinePolicy()
        )
        == -6
    )


def test_lateral_tracking_direction_rest_and_heading_wrap() -> None:
    ego = state().vehicles[0]
    assert track_lateral(replace(ego, y_m=3), 4, 0) > 0
    assert track_lateral(replace(ego, speed_mps=0), 4, 0) == 0
    assert track_lateral(replace(ego, heading_rad=2 * pi), 4, 0) == pytest.approx(0)


@pytest.mark.parametrize(
    "elapsed, expected",
    [(-1, (0, 0)), (0, (0, 0)), (1, (2, 3.75)), (2, (4, 0)), (3, (4, 0))],
)
def test_smooth_reference_endpoints(elapsed, expected) -> None:
    assert merge_reference(elapsed, 2, 4) == pytest.approx(expected)


def test_all_following_decisions_read_the_same_snapshot(monkeypatch) -> None:
    from cutin.simulation import controllers

    observed = []

    def following(vehicle, snapshot, policy):
        observed.append((vehicle.role, snapshot))
        return 0

    monkeypatch.setattr(controllers, "following_acceleration", following)
    snapshot = state()
    DrivingPolicy(ScenarioConfig(), SimulatorConfig()).act(snapshot)
    assert [role for role, _ in observed] == [Role.EGO, Role.FRONT]
    assert all(observation is snapshot for _, observation in observed)


def test_wait_and_start_use_simulation_ticks() -> None:
    policy = DrivingPolicy(ScenarioConfig(), SimulatorConfig())
    actions = policy.act(state(57))
    assert policy.report().start_tick is None
    assert actions[Role.CUT_IN].steering_rad == 0
    policy.act(state(60))
    assert policy.report().start_tick == 60
    assert policy.act(state(63))[Role.CUT_IN].steering_rad > 0


def test_duration_is_reference_not_guaranteed_completion() -> None:
    policy = DrivingPolicy(ScenarioConfig(), SimulatorConfig())
    policy.act(state(60))
    policy.observe(state(120))
    assert policy.report().completion_tick is None
    policy.observe(state(121, cut_y=4, cut_heading=0.1))
    assert policy.report().completion_tick is None
    policy.observe(state(122, cut_y=4, collided=True))
    assert policy.report().completion_tick is None
    policy.observe(state(123, cut_y=4))
    assert policy.report().completion_tick == 123


def test_completed_cut_in_switches_to_following() -> None:
    scenario = replace(ScenarioConfig(), cut_in=CutInConfig(target_speed_mps=15))
    policy = DrivingPolicy(scenario, SimulatorConfig())
    assert policy.act(state(60))[Role.CUT_IN].acceleration_mps2 == -10
    policy.observe(state(120, cut_y=4))
    snapshot = state(123, cut_y=4)
    cut_in = snapshot.vehicles[2]
    assert policy.act(snapshot)[
        Role.CUT_IN
    ].acceleration_mps2 == following_acceleration(
        cut_in, snapshot, scenario.baseline_policy
    )
