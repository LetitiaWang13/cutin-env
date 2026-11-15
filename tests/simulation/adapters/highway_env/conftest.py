"""Native straight-road and concrete-scenario fixtures, independent of the MVP."""

import numpy as np
import pytest
from highway_env.road.lane import StraightLane
from highway_env.road.road import Road, RoadNetwork

from cutin.config import CutInConfig, IDMConfig, SimulationConfig
from cutin.core.scenario import InitialVehicleState, RoadConfig, ScenarioConfig


@pytest.fixture
def road() -> Road:
    network = RoadNetwork()
    for lane in range(3):
        network.add_lane(
            "start",
            "end",
            StraightLane([0, 4 * lane], [4000, 4 * lane], speed_limit=40),
        )
    return Road(network=network, np_random=np.random.default_rng(17))


@pytest.fixture
def scenario() -> ScenarioConfig:
    return ScenarioConfig(
        road=RoadConfig(length_m=2000, lane_width_m=4, speed_limit_mps=40),
        duration_s=12,
        seed=17,
        ego=InitialVehicleState(x_m=50, lane=1, speed_mps=25),
        front=InitialVehicleState(x_m=1000, lane=1, speed_mps=25),
        cutin=InitialVehicleState(x_m=90, lane=0, speed_mps=23),
        ego_idm=IDMConfig(),
        front_idm=IDMConfig(),
        cutin_controller=CutInConfig(
            target_gap_m=25,
            lane_change_duration_s=2,
            follow=IDMConfig(desired_speed_mps=27),
        ),
    )


@pytest.fixture
def simulation_config() -> SimulationConfig:
    return SimulationConfig(backend="highway-env", physics_hz=30, controller_hz=10)
