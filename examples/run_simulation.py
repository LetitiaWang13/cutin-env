"""Run one concrete native scenario and print its record as JSON to stdout.

Usage: uv run --locked python examples/run_simulation.py > /tmp/cutin-record.json
This is direct simulation, not optimization or a new CLI/config parser.
"""

import json
from dataclasses import asdict

from cutin.config import CutInConfig, IDMConfig, SimulationConfig
from cutin.core.scenario import InitialVehicleState, RoadConfig, ScenarioConfig
from cutin.simulation.adapters.highway_env.adapter import HighwayEnvAdapter


def main() -> None:
    scenario = ScenarioConfig(
        road=RoadConfig(length_m=2000, lane_width_m=4, speed_limit_mps=40),
        duration_s=12,
        seed=17,
        ego=InitialVehicleState(x_m=50, lane=1, speed_mps=25),
        front=InitialVehicleState(x_m=1000, lane=1, speed_mps=25),
        cutin=InitialVehicleState(x_m=90, lane=0, speed_mps=23),
        ego_idm=IDMConfig(desired_speed_mps=25),
        front_idm=IDMConfig(desired_speed_mps=25),
        cutin_controller=CutInConfig(
            target_gap_m=25,
            lane_change_duration_s=2,
            follow=IDMConfig(desired_speed_mps=27),
        ),
    )
    simulation = SimulationConfig(
        backend="highway-env", physics_hz=30, controller_hz=10
    )
    record = HighwayEnvAdapter(simulation).run(scenario)
    print(json.dumps(asdict(record), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
