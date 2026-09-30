"""Gym hosting and native rendering must not change the physical trajectory."""

import json
import runpy
import sys
import tomllib
from pathlib import Path

import numpy as np
import pygame
import pytest
from gymnasium.utils.env_checker import check_env
from highway_env.envs.highway_env import HighwayEnv

from cutin.core import ControlAction, Role, ScenarioConfig, TerminationReason
from cutin.simulation import SimulatorConfig, simulate
from cutin.simulation.adapters.cut_in_env import CutInEnv
from cutin.simulation.adapters.highway_env import HighwayEnvAdapter
from cutin.simulation.adapters.highway_env_translation import (
    build_scene,
    collect_snapshot,
    road_exited,
    translate_actions,
)


@pytest.fixture
def dummy_display(monkeypatch):
    pygame.quit()
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    yield
    pygame.quit()


def joint_action():
    return {role.value: {"acceleration": 0.0, "steering": 0.0} for role in Role}


def test_standard_gym_reset_step_spaces_and_no_stock_traffic() -> None:
    env = CutInEnv(ScenarioConfig())
    try:
        assert isinstance(env, HighwayEnv)
        obs, info = env.reset(seed=42)
        assert obs.shape == (3, 7)
        assert env.observation_space.contains(obs)
        with pytest.warns(UserWarning, match="Casting input"):
            assert env.action_space.contains(joint_action())
        np.testing.assert_array_equal(obs[:, 0], [50, 110, 65])
        assert info["tick"] == 0
        obs, reward, terminated, truncated, info = env.step(joint_action())
        assert reward == 0
        assert not terminated and not truncated
        assert info["tick"] == 1
        assert info["simulation_time_s"] == pytest.approx(1 / 30)
        assert obs[0, 0] == pytest.approx(50 + 25 / 30)
        assert env.action_type is None  # Native keyboard must not overwrite commands.
        assert env.viewer is None  # Headless does not instantiate a renderer.
        env.reset(seed=42)
        assert env.steps == 0
        assert env.vehicles_by_role[Role.EGO].position[0] == 50
    finally:
        env.close()


def test_gymnasium_checker() -> None:
    env = CutInEnv(ScenarioConfig())
    try:
        # Unbounded physical coordinates/actions are intentional, not normalized
        # RL spaces. The checker warns about infinity but validates the API.
        with pytest.warns(UserWarning, match="infinity"):
            check_env(env, skip_render_check=True)
    finally:
        env.close()


def test_episode_clock_and_truncation() -> None:
    env = CutInEnv(ScenarioConfig(duration_s=0.1))
    try:
        for _ in range(3):
            _, reward, _, truncated, info = env.step(joint_action())
        assert truncated
        assert reward == 0
        assert info["tick"] == 3
        with pytest.raises(RuntimeError, match="finished episode"):
            env.step(joint_action())
        env.reset()
        env.step(joint_action(), dt_s=1 / 30)
        with pytest.raises(ValueError, match="stay fixed"):
            env.step(joint_action(), dt_s=1 / 20)
    finally:
        env.close()


def test_rgb_array_draws_actual_native_pixels_and_closes(dummy_display) -> None:
    adapter = HighwayEnvAdapter(ScenarioConfig(), render_mode="rgb_array")
    try:
        initial = adapter.render()
        assert initial.shape == (240, 1000, 3)
        assert initial.dtype == np.uint8
        for color in ((80, 180, 255), (100, 220, 130), (255, 150, 50)):
            assert np.any(np.all(initial == color, axis=2))
        commands = {role: ControlAction(0, 0) for role in Role}
        commands[Role.CUT_IN] = ControlAction(1, 0.02)
        for _ in range(10):
            adapter.step(commands, 1 / 30)
        assert not np.array_equal(adapter.render(), initial)
    finally:
        adapter.close()
    assert adapter.closed
    assert not pygame.display.get_init()


@pytest.mark.parametrize("fixture", ["baseline.toml", "unsafe.toml"])
def test_rendered_and_headless_logs_are_identical(dummy_display, fixture) -> None:
    root = Path(__file__).resolve().parents[2]
    scenario = ScenarioConfig.from_dict(
        tomllib.loads((root / "examples" / fixture).read_text())
    )
    headless = simulate(scenario, SimulatorConfig())
    adapter = HighwayEnvAdapter(scenario, render_mode="rgb_array")
    rendered_ticks = []

    def draw(snapshot):
        frame = adapter.render()
        assert frame.shape == (240, 1000, 3)
        rendered_ticks.append(snapshot.tick)

    rendered = simulate(
        scenario, SimulatorConfig(), adapter_factory=lambda _: adapter, on_step=draw
    )
    assert rendered == headless
    assert rendered_ticks == [entry.snapshot.tick for entry in rendered.entries]
    assert adapter.closed
    assert not pygame.display.get_init()


def test_native_window_close_cancels_without_advancing_again(dummy_display) -> None:
    adapter = HighwayEnvAdapter(ScenarioConfig(), render_mode="human")

    def draw(snapshot):
        if snapshot.tick == 3:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        adapter.render()
        return not adapter.closed

    log = simulate(
        ScenarioConfig(),
        SimulatorConfig(),
        adapter_factory=lambda _: adapter,
        on_step=draw,
    )
    assert log.termination_reason == TerminationReason.CANCELLED
    assert log.entries[-1].snapshot.tick == 3
    assert adapter.closed
    assert not pygame.display.get_init()


def test_native_keyboard_cannot_change_physical_actions(dummy_display) -> None:
    adapter = HighwayEnvAdapter(ScenarioConfig(), render_mode="human")
    try:
        adapter.render()
        commands = {role: ControlAction(0, 0) for role in Role}
        adapter.step(commands, 1 / 30)
        before = [dict(v.action) for v in adapter._env.vehicles_by_role.values()]
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_UP))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RIGHT))
        adapter.render()
        assert [v.action for v in adapter._env.vehicles_by_role.values()] == before
    finally:
        adapter.close()


def test_initial_observer_cancel_keeps_only_initial_state() -> None:
    adapter = HighwayEnvAdapter(ScenarioConfig())
    log = simulate(
        ScenarioConfig(),
        SimulatorConfig(),
        adapter_factory=lambda _: adapter,
        on_step=lambda snapshot: False,
    )
    assert log.termination_reason == TerminationReason.CANCELLED
    assert [entry.snapshot.tick for entry in log.entries] == [0]
    assert log.cut_in_report.start_tick is None
    assert adapter.closed


def test_observer_error_preserves_history_and_releases_viewer(dummy_display) -> None:
    adapter = HighwayEnvAdapter(ScenarioConfig(), render_mode="rgb_array")

    def draw(snapshot):
        if snapshot.tick == 2:
            raise RuntimeError("test rendering failure")
        adapter.render()

    log = simulate(
        ScenarioConfig(),
        SimulatorConfig(),
        adapter_factory=lambda _: adapter,
        on_step=draw,
    )
    assert log.termination_reason == TerminationReason.ERROR
    assert "test rendering failure" in log.diagnostic
    assert [entry.snapshot.tick for entry in log.entries] == [0, 1, 2]
    assert adapter.closed
    assert not pygame.display.get_init()


def test_visual_script_holds_terminal_scene_until_native_close(
    dummy_display, monkeypatch, tmp_path, capsys
) -> None:
    root = Path(__file__).resolve().parents[2]
    output = tmp_path / "visual.json"
    original_render = HighwayEnvAdapter.render

    def close_terminal_window(self):
        if self.episode_finished and not self.closed:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return original_render(self)

    monkeypatch.setattr(HighwayEnvAdapter, "render", close_terminal_window)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_scenario.py",
            str(root / "examples/baseline.toml"),
            "--render",
            "--output",
            str(output),
        ],
    )
    runpy.run_path(str(root / "scripts/run_scenario.py"), run_name="__main__")
    assert "Run finished. Close the native window" in capsys.readouterr().out
    data = json.loads(output.read_text())
    assert data["termination_reason"] == "horizon"
    assert data["entries"][-1]["snapshot"]["tick"] == 300
    assert not pygame.display.get_init()


class PrimitiveReferenceAdapter:
    """Pre-Gym physical stepping path, retained only as a regression oracle."""

    def __init__(self, scenario):
        self.scenario = scenario
        self.road, self.vehicles = build_scene(scenario)
        self.tick = 0

    def snapshot(self):
        return collect_snapshot(self.vehicles, self.tick)

    def step(self, actions, dt_s):
        from cutin.core import AppliedAction, LogEntry

        translated = translate_actions(actions, self.vehicles, dt_s)
        for role, v in self.vehicles.items():
            v.act(translated[role])
        self.road.act()
        self.road.step(dt_s)
        self.tick += 1
        return LogEntry(
            self.snapshot(),
            tuple(
                AppliedAction(
                    role,
                    ControlAction(
                        float(v.action["acceleration"]), float(v.action["steering"])
                    ),
                )
                for role, v in self.vehicles.items()
            ),
        )

    def road_exited(self, snapshot):
        return road_exited(snapshot, self.scenario.road)

    def backend_settings(self):
        from importlib.metadata import version

        return {"backend": "highway-env", "backend_version": version("highway-env")}

    def close(self):
        pass


@pytest.mark.parametrize("fixture", ["baseline.toml", "unsafe.toml"])
@pytest.mark.parametrize("physics_hz", [20, 30])
def test_gym_wrap_preserves_pre_gym_physics(fixture, physics_hz) -> None:
    root = Path(__file__).resolve().parents[2]
    scenario = ScenarioConfig.from_dict(
        tomllib.loads((root / "examples" / fixture).read_text())
    )
    config = SimulatorConfig(physics_hz=physics_hz)
    wrapped = simulate(scenario, config)
    reference = simulate(scenario, config, adapter_factory=PrimitiveReferenceAdapter)
    assert wrapped == reference
