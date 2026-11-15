"""Four-phase control on real highway-env dynamics, without final record schemas."""

from copy import deepcopy

import pytest
from highway_env.road.road import Road
from highway_env.vehicle.behavior import IDMVehicle

from cutin.config import CutInConfig, IDMConfig
from cutin.simulation.adapters.highway_env.controllers.cutin import (
    CutInController,
    CutInPhase,
    lateral_reference,
)
from cutin.simulation.adapters.highway_env.controllers.idm import IDMController


def make_traffic(
    road: Road,
    *,
    cut_in_x: float = 90.0,
    cut_in_speed: float = 23.0,
    config: CutInConfig | None = None,
) -> tuple[IDMController, IDMController, CutInController]:
    ego = IDMVehicle(road, [50, 4], speed=25)
    front = IDMVehicle(road, [1000, 4], speed=25)
    cut_in = IDMVehicle(road, [cut_in_x, 0], speed=cut_in_speed)
    road.vehicles = [ego, front, cut_in]
    return (
        IDMController(ego, IDMConfig()),
        IDMController(front, IDMConfig()),
        CutInController(
            cut_in,
            ego,
            ("start", "end", 1),
            config or CutInConfig(25, 2, follow=IDMConfig(desired_speed_mps=27)),
        ),
    )


def run_traffic(road, controllers, duration_s=12, physics_hz=30, controller_hz=10):
    """Test-only scheduling: all decisions, one physics step, completion observation."""
    ego, front, cut_in = controllers
    trace = []
    for tick in range(round(duration_s * physics_hz)):
        if tick % (physics_hz // controller_hz) == 0:
            ego.act()
            front.act()
            cut_in.act(tick / physics_hz)
        road.step(1 / physics_hz)
        cut_in.update_phase((tick + 1) / physics_hz)
        trace.append(
            (
                cut_in.phase,
                tuple(
                    (
                        *map(float, vehicle.position),
                        float(vehicle.heading),
                        float(vehicle.speed),
                        bool(vehicle.crashed),
                        float(vehicle.action["acceleration"]),
                    )
                    for vehicle in road.vehicles
                ),
            )
        )
        if any(vehicle.crashed for vehicle in road.vehicles):
            break
    return trace


def test_quintic_reference_endpoints_and_midpoint():
    assert lateral_reference(0, 2, 0, 4) == (0, 0)
    assert lateral_reference(2, 2, 0, 4) == (4, 0)
    assert lateral_reference(3, 2, 0, 4) == (4, 0)
    assert lateral_reference(1, 2, 0, 4) == pytest.approx((2, 3.75))
    assert lateral_reference(1, 2, 4, 0) == pytest.approx((2, -3.75))


def test_warmup_defers_a_ready_merge_without_moving_the_vehicle(road):
    _, _, cut_in = make_traffic(road, cut_in_x=80, cut_in_speed=25)
    vehicle = cut_in.vehicle
    initial_pose = (*vehicle.position, vehicle.heading, vehicle.speed)
    for time_s in (0, 1, 1.9):
        cut_in.act(time_s)
        assert cut_in.phase == CutInPhase.WARMUP
        assert vehicle.action == {"acceleration": 0, "steering": 0}
        assert (*vehicle.position, vehicle.heading, vehicle.speed) == initial_pose
    cut_in.act(2)
    assert cut_in.prepare_start_time_s == cut_in.merge_start_time_s == 2
    assert cut_in.phase == CutInPhase.MERGE


@pytest.mark.parametrize("initial_gap, next_gap", [(35, 24), (15, 26)])
def test_trigger_detects_crossing_between_control_ticks(road, initial_gap, next_gap):
    _, _, cut_in = make_traffic(road, cut_in_x=55 + initial_gap)
    cut_in.act(2)
    assert cut_in.phase == CutInPhase.PREPARE
    cut_in.vehicle.position[0] = 55 + next_gap
    cut_in.act(2.1)
    assert cut_in.phase == CutInPhase.MERGE
    assert cut_in.merge_start_time_s == 2.1
    assert abs(cut_in.relative_gap_m - 25) > cut_in.config.trigger_tolerance_m


def test_crossing_during_warmup_is_not_latched_for_a_late_merge(road):
    _, _, cut_in = make_traffic(road)
    cut_in.act(0)
    cut_in.vehicle.position[0] = 79  # Already past the target before warmup ends.
    cut_in.act(1)
    cut_in.act(2)
    cut_in.act(3)
    assert cut_in.phase == CutInPhase.PREPARE
    assert cut_in.merge_start_time_s is None


def test_physics_updates_do_not_start_preparation_or_merging(road):
    _, _, cut_in = make_traffic(road)
    cut_in.update_phase(2)
    assert cut_in.phase == CutInPhase.WARMUP
    assert cut_in.prepare_start_time_s is None

    cut_in.act(2)
    assert cut_in.phase == CutInPhase.PREPARE
    cut_in.vehicle.position[0] = 79
    cut_in.update_phase(2 + 1 / 30)
    assert cut_in.phase == CutInPhase.PREPARE
    assert cut_in.merge_start_time_s is None

    cut_in.act(2.1)
    assert cut_in.phase == CutInPhase.MERGE
    assert cut_in.merge_start_time_s == 2.1


def test_state_machine_updates_do_not_issue_actions(road):
    _, _, cut_in = make_traffic(road, cut_in_x=80)
    vehicle = cut_in.vehicle
    vehicle.action = {"acceleration": 1.0, "steering": 0.1}
    previous_action = dict(vehicle.action)

    cut_in.update_phase(2, control_tick=True)
    assert cut_in.phase == CutInPhase.MERGE
    assert vehicle.action == previous_action

    vehicle.position[1] = 4
    vehicle.on_state_update()
    cut_in.update_phase(4.1)
    assert cut_in.phase == CutInPhase.FOLLOW
    assert vehicle.action == previous_action


def test_unreachable_gap_stays_in_prepare_without_forcing_a_merge(road):
    _, _, cut_in = make_traffic(road)
    cut_in.act(2)
    cut_in.act(100)
    assert cut_in.phase == CutInPhase.PREPARE
    assert cut_in.vehicle.action == {"acceleration": 0, "steering": 0}
    assert cut_in.merge_start_time_s is None


def test_merge_sets_commands_without_teleporting_the_vehicle(road):
    _, _, cut_in = make_traffic(road, cut_in_x=80)
    vehicle = cut_in.vehicle
    before = (*vehicle.position, vehicle.heading, vehicle.speed)
    cut_in.act(2)
    cut_in.act(3)
    assert (*vehicle.position, vehicle.heading, vehicle.speed) == before
    assert vehicle.action["acceleration"] == 0
    assert 0 < vehicle.action["steering"] <= vehicle.MAX_STEERING_ANGLE


def test_stopped_vehicle_cannot_complete_a_lateral_reference(road):
    _, _, cut_in = make_traffic(road, cut_in_x=80, cut_in_speed=0)
    cut_in.act(2)
    cut_in.act(3)
    cut_in.update_phase(5)
    assert cut_in.vehicle.action == {"acceleration": 0, "steering": 0}
    assert cut_in.phase == CutInPhase.MERGE
    assert cut_in.merge_completion_time_s is None


def test_elapsed_reference_time_does_not_fake_physical_completion(road):
    _, _, cut_in = make_traffic(road, cut_in_x=80)
    cut_in.act(2)
    cut_in.update_phase(5)
    assert cut_in.phase == CutInPhase.MERGE
    assert cut_in.merge_completion_time_s is None


def test_follow_handoff_preserves_vehicle_state_and_uses_native_idm(road, monkeypatch):
    _, _, cut_in = make_traffic(road, cut_in_x=80)
    vehicle = cut_in.vehicle
    native_act = vehicle.act
    native_calls = []

    def observe_native_act():
        native_calls.append(cut_in.phase)
        native_act()

    monkeypatch.setattr(vehicle, "act", observe_native_act)
    for time_s in (0, 2, 2.5):
        cut_in.act(time_s)
    assert not native_calls
    vehicle.position[1] = 4
    vehicle.heading = 0
    vehicle.on_state_update()
    before = (*vehicle.position, vehicle.heading, vehicle.speed)
    cut_in.update_phase(4.1)
    assert cut_in.phase == CutInPhase.FOLLOW
    assert cut_in.merge_completion_time_s == 4.1
    assert vehicle is road.vehicles[2]
    assert (*vehicle.position, vehicle.heading, vehicle.speed) == before
    assert vehicle.target_lane_index == ("start", "end", 1)
    assert vehicle.target_speed == 27
    assert not vehicle.enable_lane_change
    cut_in.act(4.1)
    assert native_calls == [CutInPhase.FOLLOW]
    assert vehicle.action["acceleration"] > 0


def test_follow_brakes_for_front_instead_of_holding_the_merge_speed(road):
    _, front, cut_in = make_traffic(road, cut_in_x=80)
    cut_in.act(2)
    cut_in.vehicle.position[1] = 4
    cut_in.vehicle.on_state_update()
    cut_in.update_phase(4.1)
    front.vehicle.position[0] = cut_in.vehicle.position[0] + 15
    front.vehicle.speed = 10
    front.vehicle.on_state_update()
    cut_in.act(4.1)
    assert cut_in.vehicle.action["acceleration"] < 0


def test_collision_does_not_become_a_completed_maneuver(road):
    _, _, cut_in = make_traffic(road, cut_in_x=80)
    cut_in.act(2)
    cut_in.vehicle.position[1] = 4
    cut_in.vehicle.crashed = True
    cut_in.update_phase(5)
    cut_in.act(5)
    assert cut_in.phase == CutInPhase.MERGE
    assert cut_in.merge_completion_time_s is None


def test_shared_native_road_runs_all_phases_and_ego_reacts(road):
    controllers = make_traffic(road)
    cut_in = controllers[2]
    trace = run_traffic(road, controllers)
    assert {phase for phase, _ in trace} == set(CutInPhase)
    assert len(trace) == 360
    assert cut_in.prepare_start_time_s == 2
    assert cut_in.merge_start_time_s > 2
    assert cut_in.merge_completion_time_s >= cut_in.merge_start_time_s + 2
    assert cut_in.phase == CutInPhase.FOLLOW
    assert cut_in.vehicle.target_lane_index == ("start", "end", 1)
    assert cut_in.vehicle.position[1] == pytest.approx(4, abs=0.2)
    assert cut_in.vehicle.speed > cut_in.initial_speed_mps
    assert all(
        states[2][1] == 0
        for phase, states in trace
        if phase in (CutInPhase.WARMUP, CutInPhase.PREPARE)
    )
    assert all(
        states[2][3] == pytest.approx(23)
        for phase, states in trace
        if phase != CutInPhase.FOLLOW
    )
    assert any(
        states[0][5] < -0.5 for phase, states in trace if phase == CutInPhase.MERGE
    )


def test_same_setup_and_clock_repeat_the_full_native_trace(road):
    other_road = deepcopy(road)
    first = run_traffic(road, make_traffic(road))
    second = run_traffic(other_road, make_traffic(other_road))
    assert first == second


@pytest.mark.parametrize("target_gap", [20, 30])
def test_target_gap_controls_observed_merge_initiation(road, target_gap):
    controllers = make_traffic(road, config=CutInConfig(target_gap, 2))
    cut_in = controllers[2]
    for tick in range(300):
        if tick % 3 == 0:
            controllers[0].act()
            controllers[1].act()
            cut_in.act(tick / 30)
            if cut_in.phase == CutInPhase.MERGE:
                break
        road.step(1 / 30)
    assert cut_in.phase == CutInPhase.MERGE
    assert cut_in.relative_gap_m == pytest.approx(target_gap, abs=0.2)


def test_reference_duration_changes_actual_maneuver_time(road):
    other_road = deepcopy(road)
    fast = make_traffic(road, config=CutInConfig(25, 2))
    slow = make_traffic(other_road, config=CutInConfig(25, 3))
    run_traffic(road, fast)
    run_traffic(other_road, slow)
    fast_time = fast[2].merge_completion_time_s - fast[2].merge_start_time_s
    slow_time = slow[2].merge_completion_time_s - slow[2].merge_start_time_s
    assert slow_time > fast_time
