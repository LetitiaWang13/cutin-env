"""Simulation inputs, separate from state outputs. Values use SI units."""

from dataclasses import dataclass

from cutin_mvp.core.state import Role


@dataclass(frozen=True)
class ControlAction:
    acceleration_mps2: float
    steering_rad: float


@dataclass(frozen=True)
class AppliedAction:
    """Role-keyed action after the simulation applies actuator bounds."""

    role: Role
    action: ControlAction
