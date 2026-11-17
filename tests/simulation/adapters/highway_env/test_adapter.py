"""Real ScenarioConfig -> native dynamics -> detached ScenarioRecord integration."""

import json
from dataclasses import asdict, replace
from importlib.metadata import version

import pytest
from highway_env.road.road import Road

from cutin.core.scenario import InitialVehicleState
from cutin.simulation.adapters.highway_env.adapter import (
    CONTROLLER_VERSION,
    HighwayEnvAdapter,
)
from cutin.simulation.adapters.highway_env.controllers.cutin import CutInController
from cutin.simulation.adapters.highway_env.controllers.idm import IDMController


def test_complete_scenario_records_all_ticks_and_four_phases(
    scenario, simulation_config
):
    before = asdict(scenario)
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    assert asdict(scenario) == before
    assert record.scenario == scenario
    assert record.simulation == simulation_config
    assert record.backend_version == version("highway-env")
    assert record.controller_version == CONTROLLER_VERSION
    assert record.termination_reason == "horizon"
    assert [frame.tick for frame in record.frames] == list(range(361))
    assert {frame.cutin_phase for frame in record.frames} == {
        "warmup",
        "prepare",
        "merge",
        "follow",
    }
    assert record.prepare_start_time_s == 2
    assert record.merge_start_time_s > 2
    assert record.merge_completion_time_s >= record.merge_start_time_s + 2
    initial, final = record.frames[0], record.frames[-1]
    assert initial.ego.x_m == 50 and initial.ego.y_m == 4
    assert initial.front.x_m == 1000 and initial.front.y_m == 4
    assert initial.cutin.x_m == 90 and initial.cutin.y_m == 0
    assert initial.cutin.heading_rad == 0
    assert initial.cutin.speed_mps == 23
    assert initial.cutin.length_m == 5 and initial.cutin.width_m == 2
    assert final.cutin_phase == "follow"
    assert final.cutin.y_m == pytest.approx(4, abs=0.2)
    assert final.cutin.speed_mps > initial.cutin.speed_mps
    assert not any(
        vehicle.collided for vehicle in (final.ego, final.front, final.cutin)
    )


def test_initial_actions_are_absent_and_held_commands_are_recorded(
    scenario, simulation_config
):
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    initial = record.frames[0]
    for vehicle in (initial.ego, initial.front, initial.cutin):
        assert vehicle.acceleration_mps2 is None
        assert vehicle.steering_rad is None
    for frame in record.frames[1:]:
        for vehicle in (frame.ego, frame.front, frame.cutin):
            assert vehicle.acceleration_mps2 is not None
            assert vehicle.steering_rad is not None
    first_commands = [frame.ego.acceleration_mps2 for frame in record.frames[1:4]]
    assert first_commands == [first_commands[0]] * 3
    assert record.frames[4].ego.acceleration_mps2 != first_commands[0]
    assert initial.ego.x_m == 50  # Later physics must not mutate an earlier frame.
    assert record.frames[1].ego.x_m > initial.ego.x_m


def test_adapter_owns_one_physics_step_and_control_clock(
    scenario, simulation_config, monkeypatch
):
    step_durations = []
    cutin_decision_times = []
    native_decision_ticks = {}
    original_step = Road.step
    original_cutin_act = CutInController.act
    original_idm_act = IDMController.act

    def step(road, dt):
        step_durations.append(dt)
        original_step(road, dt)

    def cutin_act(controller, time_s):
        cutin_decision_times.append(time_s)
        original_cutin_act(controller, time_s)

    def idm_act(controller):
        native_decision_ticks.setdefault(id(controller), []).append(len(step_durations))
        original_idm_act(controller)

    def autonomous_road_act(road):
        pytest.fail("Road.act would overwrite adapter-scheduled controller actions")

    monkeypatch.setattr(Road, "step", step)
    monkeypatch.setattr(Road, "act", autonomous_road_act)
    monkeypatch.setattr(CutInController, "act", cutin_act)
    monkeypatch.setattr(IDMController, "act", idm_act)
    HighwayEnvAdapter(simulation_config).run(scenario)
    assert step_durations == [1 / 30] * 360
    assert cutin_decision_times == [tick / 30 for tick in range(0, 360, 3)]
    assert len(native_decision_ticks) == 3  # Ego, front, and the cut-in FOLLOW policy.
    assert sum(len(ticks) == 120 for ticks in native_decision_ticks.values()) == 2
    assert all(
        tick % 3 == 0 for ticks in native_decision_ticks.values() for tick in ticks
    )


def test_reusing_adapter_creates_fresh_episodes_and_repeatable_records(
    scenario, simulation_config
):
    adapter = HighwayEnvAdapter(simulation_config)
    first = adapter.run(scenario)
    first_data = asdict(first)
    other = replace(scenario, cutin=InitialVehicleState(110, 0, 23))
    different = adapter.run(other)
    repeated = adapter.run(scenario)
    assert different != first
    assert asdict(first) == first_data
    assert repeated == first


def test_unreachable_target_stays_unfinished_at_horizon(scenario, simulation_config):
    scenario = replace(scenario, cutin=InitialVehicleState(90, 0, 25))
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    assert record.termination_reason == "horizon"
    assert record.frames[-1].cutin_phase == "prepare"
    assert record.prepare_start_time_s == 2
    assert record.merge_start_time_s is None
    assert record.merge_completion_time_s is None


def test_short_horizon_preserves_an_unfinished_merge(scenario, simulation_config):
    scenario = replace(
        scenario,
        duration_s=1,
        cutin=InitialVehicleState(80, 0, 23),
        cutin_controller=replace(
            scenario.cutin_controller, warmup_s=0, lane_change_duration_s=20
        ),
    )
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    assert record.frames[-1].tick == 30
    assert record.frames[-1].cutin_phase == "merge"
    assert record.prepare_start_time_s == record.merge_start_time_s == 0
    assert record.merge_completion_time_s is None


def test_native_idm_observes_explicit_road_speed_limit_and_lane_width(
    scenario, simulation_config
):
    scenario = replace(
        scenario,
        duration_s=0.1,
        road=replace(scenario.road, lane_width_m=3.5, speed_limit_mps=10),
    )
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    assert record.frames[0].ego.y_m == record.frames[0].front.y_m == 3.5
    assert record.frames[0].cutin.y_m == 0
    assert record.frames[1].ego.acceleration_mps2 < 0
    assert record.frames[-1].ego.speed_mps < scenario.ego.speed_mps


def test_collision_stops_at_the_first_collided_frame(scenario, simulation_config):
    scenario = replace(scenario, front=InitialVehicleState(52, 1, 25))
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    assert record.termination_reason == "collision"
    assert len(record.frames) < 361
    final = record.frames[-1]
    assert any(vehicle.collided for vehicle in (final.ego, final.front, final.cutin))
    assert all(
        not vehicle.collided
        for frame in record.frames[:-1]
        for vehicle in (frame.ego, frame.front, frame.cutin)
    )
    assert final.tick == len(record.frames) - 1
    assert record.merge_completion_time_s is None


def test_follow_phase_and_applied_action_have_distinct_timing(
    scenario, simulation_config
):
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    completed_tick = round(
        record.merge_completion_time_s * simulation_config.physics_hz
    )
    assert record.frames[completed_tick].cutin_phase == "follow"
    assert record.frames[completed_tick].cutin.acceleration_mps2 == 0
    interval = simulation_config.physics_hz // simulation_config.controller_hz
    next_command_tick = ((completed_tick + interval - 1) // interval) * interval
    assert all(
        frame.cutin.acceleration_mps2 == 0
        for frame in record.frames[completed_tick : next_command_tick + 1]
    )
    assert record.frames[next_command_tick + 1].cutin.acceleration_mps2 > 0


def test_record_exports_and_reloads_as_plain_json(
    scenario, simulation_config, tmp_path
):
    record = HighwayEnvAdapter(simulation_config).run(scenario)
    document = asdict(record)
    path = tmp_path / "record.json"
    path.write_text(json.dumps(document, allow_nan=False), encoding="utf-8")
    loaded = json.loads(path.read_text(encoding="utf-8"))
    assert loaded == document
    assert loaded["frames"][0]["cutin"]["acceleration_mps2"] is None
    assert loaded["scenario"]["cutin_controller"]["follow"]["desired_speed_mps"] == 27
    assert loaded["simulation"]["physics_hz"] == 30


def test_fractional_horizon_ends_on_the_next_physics_tick(scenario, simulation_config):
    record = HighwayEnvAdapter(simulation_config).run(
        replace(scenario, duration_s=0.05)
    )
    assert [frame.tick for frame in record.frames] == [0, 1, 2]
    actual_end_s = record.frames[-1].tick / simulation_config.physics_hz
    assert actual_end_s >= record.scenario.duration_s
    assert actual_end_s < record.scenario.duration_s + 1 / simulation_config.physics_hz
