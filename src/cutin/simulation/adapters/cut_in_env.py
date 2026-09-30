"""Custom HighwayEnv scene with joint physical actions and native rendering.

Gym is an environment/rendering interface here, not an RL training integration.
One step is one physics tick. Stock traffic, rewards, and multi-frame scheduling
are bypassed; Road/Vehicle dynamics remain highway-env's implementation.
"""

from dataclasses import replace
from math import isclose, isfinite

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from highway_env.envs.common.abstract import AbstractEnv
from highway_env.envs.common.graphics import EnvViewer
from highway_env.envs.highway_env import HighwayEnv

from cutin.core import Role, ScenarioConfig, validate_scenario
from cutin.simulation.adapters.highway_env_translation import (
    build_scene,
    collect_snapshot,
    road_exited,
)


class CutInEnv(HighwayEnv):
    """Physical joint actions: {role: {acceleration: m/s^2, steering: rad}}.

    Observation rows follow Role order; columns are x, y, heading, speed, length,
    width, collided. Reward is always zero and is never used as a safety metric.
    """

    def __init__(self, scenario: ScenarioConfig, *, render_mode: str | None = None):
        validate_scenario(scenario)
        if render_mode not in (None, "human", "rgb_array"):
            raise ValueError("render_mode must be None, human, or rgb_array")
        self.scenario = scenario
        self.episode_finished = False
        # A copy avoids changing class-level FPS metadata across environments.
        self.metadata = dict(type(self).metadata)
        super().__init__(render_mode=render_mode)

    @classmethod
    def default_config(cls) -> dict:
        # Keep the native renderer's options, not HighwayEnv's traffic/RL defaults.
        return {
            **AbstractEnv.default_config(),
            "simulation_frequency": 30,
            "policy_frequency": 30,
            "screen_width": 1000,
            "screen_height": 240,
            "centering_position": [0.25, 0.5],
            "scaling": 6.0,
            "manual_control": False,
            "render_agent": False,
            "real_time_rendering": True,
        }

    def define_spaces(self) -> None:
        # No stock ActionType: native keyboard events must not drive a vehicle.
        self.action_type = None
        self.observation_type = None
        self.action_space = spaces.Dict(
            {
                role.value: spaces.Dict(
                    {
                        "acceleration": spaces.Box(
                            -np.inf, np.inf, shape=(), dtype=np.float64
                        ),
                        "steering": spaces.Box(
                            -np.inf, np.inf, shape=(), dtype=np.float64
                        ),
                    }
                )
                for role in Role
            }
        )
        self.observation_space = spaces.Box(
            -np.inf, np.inf, shape=(3, 7), dtype=np.float64
        )

    def _observation(self) -> np.ndarray:
        return np.array(
            [
                [
                    v.position[0],
                    v.position[1],
                    v.heading,
                    v.speed,
                    v.LENGTH,
                    v.WIDTH,
                    v.crashed,
                ]
                for v in self.vehicles_by_role.values()
            ],
            dtype=np.float64,
        )

    def _info(self, obs=None, action=None) -> dict:
        return {"tick": self.steps, "simulation_time_s": self.time}

    def reset(self, *, seed=None, options=None):
        if options:
            raise ValueError(
                "configure the scenario via ScenarioConfig, not reset options"
            )
        seed = self.scenario.seed if seed is None else seed
        gym.Env.reset(self, seed=seed)
        self.road, self.vehicles_by_role = build_scene(
            replace(self.scenario, seed=seed)
        )
        colors = {
            Role.EGO: (80, 180, 255),
            Role.FRONT: (100, 220, 130),
            Role.CUT_IN: (255, 150, 50),
        }
        for role, vehicle in self.vehicles_by_role.items():
            vehicle.color = colors[role]
        self.controlled_vehicles = [self.vehicles_by_role[Role.EGO]]
        self.steps, self.time = 0, 0.0
        self.done = self.episode_finished = False
        self.metadata["render_fps"] = round(self.config["simulation_frequency"])
        if self.render_mode == "human":
            self.render()
        return self._observation(), self._info()

    def step(self, action, *, dt_s: float | None = None):
        """Standard Gym step; the adapter can additionally supply its fixed dt."""
        if self.done:
            raise RuntimeError("environment is closed")
        if self.episode_finished:
            raise RuntimeError("reset before stepping a finished episode")
        dt_s = 1 / self.config["simulation_frequency"] if dt_s is None else dt_s
        if not isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        if self.steps and not isclose(
            dt_s, 1 / self.config["simulation_frequency"], rel_tol=0, abs_tol=1e-12
        ):
            raise ValueError("physics dt must stay fixed within an episode")
        if set(action) != {role.value for role in Role}:
            raise ValueError("provide joint physical actions for ego/front/cut_in")
        # Validate all inputs before mutating backend actions or physics.
        physical = {
            role: {
                name: float(action[role.value][name])
                for name in ("acceleration", "steering")
            }
            for role in Role
        }
        if not all(
            isfinite(value)
            for command in physical.values()
            for value in command.values()
        ):
            raise ValueError("actions must be finite")
        for role, vehicle in self.vehicles_by_role.items():
            vehicle.act(physical[role])
        self.road.act()
        self.road.step(dt_s)
        self.steps += 1
        self.time = self.steps * dt_s  # Integer physics tick remains authoritative.
        self.config["simulation_frequency"] = self.config["policy_frequency"] = 1 / dt_s
        self.metadata["render_fps"] = round(1 / dt_s)
        snapshot = collect_snapshot(self.vehicles_by_role, self.steps)
        terminated = any(
            vehicle.collided for vehicle in snapshot.vehicles
        ) or road_exited(snapshot, self.scenario.road)
        truncated = self.time >= self.scenario.duration_s
        self.episode_finished = terminated or truncated
        if self.render_mode == "human":
            self.render()
        return self._observation(), 0.0, terminated, truncated, self._info()

    def render(self):
        if self.done or self.render_mode is None:
            return None
        if self.viewer is None:
            self.viewer = EnvViewer(self)
            # Upstream disables drawing with SDL dummy, even for off-screen
            # surfaces. RGB rendering is valid there and must draw actual pixels.
            if self.render_mode == "rgb_array":
                self.viewer.enabled = True
        return super().render()
