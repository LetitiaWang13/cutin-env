"""Native IDM configuration and real following behavior."""

from highway_env.road.road import Road
from highway_env.vehicle.behavior import IDMVehicle

from cutin.config import IDMConfig
from cutin.simulation.adapters.highway_env.controllers.idm import IDMController


def test_parameters_are_per_vehicle_and_do_not_change_native_defaults(road: Road):
    native_headway = IDMVehicle.TIME_WANTED
    ego = IDMVehicle(road, [50, 4], speed=20)
    front = IDMVehicle(road, [110, 4], speed=25)
    config = IDMConfig(
        desired_speed_mps=27,
        time_headway_s=2,
        minimum_gap_m=3,
        comfortable_acceleration_mps2=2,
        comfortable_braking_mps2=4,
        acceleration_limit_mps2=7,
        speed_exponent=3,
    )
    IDMController(ego, config)
    IDMController(front, IDMConfig())
    assert ego.target_speed == 27
    assert ego.TIME_WANTED == 2
    assert ego.DISTANCE_WANTED == 3 + ego.LENGTH
    assert ego.COMFORT_ACC_MAX == 2
    assert ego.COMFORT_ACC_MIN == -4
    assert ego.ACC_MAX == 7
    assert ego.DELTA == 3
    assert not ego.enable_lane_change and not front.enable_lane_change
    assert front.TIME_WANTED == IDMConfig().time_headway_s
    assert IDMVehicle.TIME_WANTED == native_headway


def test_free_vehicle_uses_native_acceleration_and_lane_tracking(road: Road):
    vehicle = IDMVehicle(road, [50, 4], speed=20)
    road.vehicles = [vehicle]
    controller = IDMController(vehicle, IDMConfig(desired_speed_mps=25))
    controller.act()
    assert vehicle.action["acceleration"] > 0
    assert vehicle.action["steering"] == 0
    expected = dict(vehicle.action)
    IDMVehicle.act(vehicle)
    assert vehicle.action == expected


def test_native_idm_brakes_for_a_slower_front_vehicle(road: Road):
    ego = IDMVehicle(road, [50, 4], speed=25)
    front = IDMVehicle(road, [65, 4], speed=10)
    road.vehicles = [ego, front]
    controller = IDMController(ego, IDMConfig())
    controller.act()
    assert ego.action["acceleration"] < 0
    before = ego.speed
    road.step(1 / 30)
    assert ego.speed < before


def test_zero_desired_speed_is_not_replaced_by_initial_speed(road: Road):
    vehicle = IDMVehicle(road, [50, 4], speed=10)
    road.vehicles = [vehicle]
    controller = IDMController(vehicle, IDMConfig(desired_speed_mps=0))
    controller.act()
    assert vehicle.target_speed == 0
    assert vehicle.action["acceleration"] < 0
