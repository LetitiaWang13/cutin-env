"""Simulator settings are validated once at the adapter boundary."""

from dataclasses import replace

import pytest

from cutin.simulation import SimulatorConfig
from cutin.simulation.config import validate_simulator_config


def test_defaults_and_serialization() -> None:
    config = SimulatorConfig()
    validate_simulator_config(config)
    assert config.to_dict()["physics_hz"] == 30
    assert config.to_dict()["backend"] == "highway"


@pytest.mark.parametrize(
    "changes",
    [
        {"backend": "carla"},
        {"physics_hz": 0},
        {"physics_hz": True},
        {"controller_hz": 0},
        {"controller_hz": 7},
        {"controller_hz": 10.0},
        {"max_acceleration_mps2": 0},
        {"max_braking_mps2": -1},
        {"max_steering_rad": float("nan")},
        {"max_steering_rad": float("inf")},
        {"max_steering_rad": 2},
        {"max_braking_mps2": True},
    ],
)
def test_invalid_settings(changes: dict) -> None:
    with pytest.raises(ValueError):
        validate_simulator_config(replace(SimulatorConfig(), **changes))
