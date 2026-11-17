"""Backend-independent motion records, not metrics or resumable checkpoints."""

from dataclasses import dataclass

from cutin.config import SimulationConfig
from cutin.core.scenario import ScenarioConfig


@dataclass
class VehicleState:
    """Post-step pose and the actual action used in the interval ending here.

    The initial frame has no preceding interval, so its action fields are None.
    Dimensions are copied from the executed vehicle rather than assumed by replay.
    """

    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    length_m: float
    width_m: float
    collided: bool
    acceleration_mps2: float | None
    steering_rad: float | None


@dataclass
class Frame:
    """One physics tick, including tick zero; seconds = tick / physics_hz.

    cutin_phase is sampled after the phase update. It may already be FOLLOW while
    the recorded interval still used MERGE commands until the next control tick.
    """

    tick: int
    ego: VehicleState
    front: VehicleState
    cutin: VehicleState
    cutin_phase: str


@dataclass
class ScenarioRecord:
    """A complete or collision-truncated episode, with resolved execution inputs.

    Unobserved phase events remain None. No TTC/LCT/robustness is cached here;
    evaluation computes those from the trajectory and event times separately.
    """

    scenario: ScenarioConfig
    simulation: SimulationConfig
    backend_version: str
    controller_version: str

    frames: list[Frame]
    termination_reason: str

    prepare_start_time_s: float | None
    merge_start_time_s: float | None
    merge_completion_time_s: float | None
