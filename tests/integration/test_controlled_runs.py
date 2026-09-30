"""Configured maneuvers execute on real highway-env dynamics."""

import tomllib
from dataclasses import replace
from pathlib import Path

import pytest

from cutin.core import CutInConfig, Role, ScenarioConfig, TerminationReason
from cutin.simulation import SimulatorConfig, simulate


@pytest.fixture(scope="module")
def baseline():
    return simulate(ScenarioConfig(), SimulatorConfig())


def test_baseline_completes_physical_merge_without_collision(baseline) -> None:
    assert baseline.termination_reason == TerminationReason.HORIZON
    assert not any(
        vehicle.collided
        for entry in baseline.entries
        for vehicle in entry.snapshot.vehicles
    )
    assert baseline.cut_in_report.start_tick == 60
    # Actual tracking completion is later than the 2-second reference endpoint.
    assert baseline.cut_in_report.completion_tick > 120
    assert baseline.entries[60].snapshot.vehicles_for_role(Role.CUT_IN)[0].y_m == 0
    final = baseline.entries[-1].snapshot
    assert final.vehicles_for_role(Role.CUT_IN)[0].y_m == pytest.approx(4, abs=0.2)
    assert final.vehicles_for_role(Role.EGO)[0].y_m == pytest.approx(4)
    assert final.vehicles_for_role(Role.FRONT)[0].y_m == pytest.approx(4)
    assert all(len(entry.applied_actions) == 3 for entry in baseline.entries[1:])


def test_unsafe_example_collides_with_ego_not_front() -> None:
    root = Path(__file__).resolve().parents[2]
    scenario = ScenarioConfig.from_dict(
        tomllib.loads((root / "examples/unsafe.toml").read_text())
    )
    log = simulate(scenario, SimulatorConfig())
    assert log.termination_reason == TerminationReason.COLLISION
    terminal = log.entries[-1].snapshot
    assert terminal.vehicles_for_role(Role.EGO)[0].collided
    assert terminal.vehicles_for_role(Role.CUT_IN)[0].collided
    assert not terminal.vehicles_for_role(Role.FRONT)[0].collided
    assert log.cut_in_report.completion_tick is None


def test_policy_state_is_fresh_and_runs_are_repeatable(baseline) -> None:
    assert simulate(ScenarioConfig(), SimulatorConfig()) == baseline


def test_late_trigger_reports_unstarted_maneuver() -> None:
    scenario = replace(ScenarioConfig(), cut_in=CutInConfig(start_time_s=20))
    log = simulate(scenario, SimulatorConfig())
    assert log.termination_reason == TerminationReason.HORIZON
    assert log.cut_in_report.start_tick is None
    assert log.cut_in_report.completion_tick is None
    assert log.entries[-1].snapshot.vehicles_for_role(Role.CUT_IN)[0].y_m == 0


def test_start_quantizes_to_next_control_tick() -> None:
    scenario = replace(ScenarioConfig(), cut_in=CutInConfig(start_time_s=2.05))
    log = simulate(scenario, SimulatorConfig())
    assert log.cut_in_report.start_tick == 63  # 2.1 seconds, not 2.05


def test_timing_and_speed_change_executed_motion(baseline) -> None:
    later = simulate(
        replace(ScenarioConfig(), cut_in=CutInConfig(start_time_s=3)), SimulatorConfig()
    )
    assert later.cut_in_report.start_tick == 90
    assert later.entries[75].snapshot.vehicles_for_role(Role.CUT_IN)[0].y_m == 0
    assert baseline.entries[75].snapshot.vehicles_for_role(Role.CUT_IN)[0].y_m > 0
    slower = simulate(
        replace(ScenarioConfig(), cut_in=CutInConfig(target_speed_mps=20)),
        SimulatorConfig(),
    )
    assert (
        slower.entries[45].snapshot.vehicles_for_role(Role.CUT_IN)[0].speed_mps
        < baseline.entries[45].snapshot.vehicles_for_role(Role.CUT_IN)[0].speed_mps
    )


def test_initial_position_changes_executed_trajectory(baseline) -> None:
    scenario = ScenarioConfig()
    shifted = replace(
        scenario,
        initial=replace(
            scenario.initial, cut_in=replace(scenario.initial.cut_in, x_m=80)
        ),
    )
    log = simulate(shifted, SimulatorConfig())
    x = log.entries[45].snapshot.vehicles_for_role(Role.CUT_IN)[0].x_m
    baseline_x = baseline.entries[45].snapshot.vehicles_for_role(Role.CUT_IN)[0].x_m
    assert x - baseline_x == pytest.approx(15)


def test_duration_changes_motion_and_completion() -> None:
    fast = simulate(
        replace(ScenarioConfig(), cut_in=CutInConfig(lane_change_duration_s=1)),
        SimulatorConfig(),
    )
    slow = simulate(
        replace(ScenarioConfig(), cut_in=CutInConfig(lane_change_duration_s=3)),
        SimulatorConfig(),
    )
    assert (
        fast.entries[90].snapshot.vehicles_for_role(Role.CUT_IN)[0].y_m
        > slow.entries[90].snapshot.vehicles_for_role(Role.CUT_IN)[0].y_m
    )
    assert fast.cut_in_report.completion_tick < slow.cut_in_report.completion_tick
