"""Web-only initial clearance buffer; core/search admissibility is unchanged."""

from itertools import combinations

from cutin.core import ScenarioConfig

POSITION_CLEARANCE_M = 0.1


def validate_initial_clearance(scenario: ScenarioConfig) -> None:
    vehicles = [
        (name, getattr(scenario.initial, name)) for name in ("ego", "front", "cut_in")
    ]
    required = scenario.vehicles.length_m + POSITION_CLEARANCE_M
    for (name_a, a), (name_b, b) in combinations(vehicles, 2):
        lateral = abs(a.lane - b.lane) * scenario.road.lane_width_m
        separation = abs(a.x_m - b.x_m)
        if lateral <= scenario.vehicles.width_m and separation + 1e-9 < required:
            raise ValueError(
                f"{name_a}/{name_b}: center separation {separation:.9f} m; "
                f"require at least {required:.3f} m "
                f"(vehicle length {scenario.vehicles.length_m:.3f} m "
                f"+ clearance {POSITION_CLEARANCE_M:.3f} m). "
                "Move these vehicles farther apart."
            )
