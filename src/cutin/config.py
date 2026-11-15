"""Controller/execution settings and the remaining user-facing config sections."""

from dataclasses import dataclass, field


@dataclass
class GenerationConfig:
    """Generator choice, fixed conditions, aggressiveness, and optimizer settings."""


@dataclass
class SimulationConfig:
    """Fixed execution clock; physics_hz is an integer multiple of controller_hz."""

    # Backend identifier; the current concrete adapter executes highway-env.
    backend: str
    # Physics steps and recorded frames per simulated second.
    physics_hz: int
    # Driving decisions per simulated second; actions are held between decisions.
    controller_hz: int


@dataclass(frozen=True)
class IDMConfig:
    """Native IDM settings; defaults are illustrative, not safety-calibrated.

    acceleration_limit_mps2 bounds both acceleration and braking commands.
    minimum_gap_m is a bumper gap for the equal-length vehicles used here.
    """

    # Desired free-road speed in m/s; native IDM also respects the lane speed limit.
    desired_speed_mps: float = 25.0
    # Desired following time gap in seconds, not a perception/reaction delay.
    time_headway_s: float = 1.5
    # Base standstill bumper gap in metres; not a guaranteed minimum separation.
    minimum_gap_m: float = 5.0
    # Positive comfortable acceleration magnitude used by the IDM formula, in m/s².
    comfortable_acceleration_mps2: float = 3.0
    # Positive comfortable braking magnitude, mapped to native COMFORT_ACC_MIN < 0.
    comfortable_braking_mps2: float = 5.0
    # Symmetric command limit: acceleration is clipped to ±this value, in m/s².
    acceleration_limit_mps2: float = 6.0
    # Dimensionless exponent controlling the free-road speed/acceleration curve.
    speed_exponent: float = 4.0


@dataclass(frozen=True)
class CutInConfig:
    """Constant-speed preparation and a timed lateral reference on a straight road.

    target_gap_m is the cut-in rear bumper minus ego's front bumper at merge
    initiation. Initial position/speed belong to the vehicle, not this config.
    Actual completion is observed, not guaranteed by lane_change_duration_s.
    """

    # Requested longitudinal bumper gap at merge start; positive is ahead of ego.
    target_gap_m: float
    # Planned lateral-reference duration in seconds, not the measured actual LCT.
    lane_change_duration_s: float
    # Time from episode start before preparation can begin; keep lane/speed until then.
    warmup_s: float = 2.0
    # Gap-error tolerance in metres for triggering MERGE; crossing also triggers it.
    trigger_tolerance_m: float = 0.1
    # Maximum lateral distance from the destination lane centre for completion, in m.
    completion_position_tolerance_m: float = 0.2
    # Maximum heading error relative to the destination lane for completion, in rad.
    completion_heading_tolerance_rad: float = 0.05
    # Native IDM settings for FOLLOW, including its independent desired cruise speed.
    # Its acceleration limit also bounds speed-control commands in earlier phases.
    follow: IDMConfig = field(default_factory=IDMConfig)


@dataclass
class EvaluationConfig:
    """Requested metrics and their evaluation parameters."""


@dataclass
class InterfaceConfig:
    """CLI/Web, saving, and playback preferences."""


@dataclass
class UserConfig:
    # Generator selection, fixed scenario conditions, targets, and optimizer settings.
    generation: GenerationConfig
    # Simulation backend selection and backend-specific execution settings.
    simulation: SimulationConfig
    # Requested metrics and the parameters used to calculate them.
    evaluation: EvaluationConfig
    # User-facing CLI/Web, artifact saving, and playback preferences.
    interface: InterfaceConfig
