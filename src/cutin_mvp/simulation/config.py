"""Numerical/actuator settings, separate from the scenario being tested."""

from dataclasses import asdict, dataclass
from math import isfinite, pi


@dataclass(frozen=True)
class SimulatorConfig:
    backend: str = "highway-env"
    physics_hz: int = 30
    controller_hz: int = 10
    max_acceleration_mps2: float = 3.0
    max_braking_mps2: float = 6.0
    max_steering_rad: float = 0.5

    def to_dict(self) -> dict[str, str | int | float]:
        return asdict(self)


def validate_simulator_config(config: SimulatorConfig) -> None:
    """Validate at the runner boundary, not inside a backend adapter."""
    if config.backend != "highway-env":
        raise ValueError("only the highway-env backend is supported")
    for name in ("physics_hz", "controller_hz"):
        value = getattr(config, name)
        if type(value) is not int or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
    if config.physics_hz % config.controller_hz:
        raise ValueError("controller_hz must divide physics_hz")
    for name in ("max_acceleration_mps2", "max_braking_mps2", "max_steering_rad"):
        value = getattr(config, name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{name} must be finite and positive")
    if config.max_steering_rad >= pi / 2:
        raise ValueError("max_steering_rad must be below pi/2 for the bicycle model")
