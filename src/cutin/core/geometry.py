"""Shared footprint convention for control, road bounds, and safety metrics."""

from math import cos, sin

from cutin.core.state import VehicleState


def half_extents(vehicle: VehicleState) -> tuple[float, float]:
    """Conservative world-axis half extents of a heading-rotated rectangle."""
    c, s = abs(cos(vehicle.heading_rad)), abs(sin(vehicle.heading_rad))
    return (
        (vehicle.length_m * c + vehicle.width_m * s) / 2,
        (vehicle.length_m * s + vehicle.width_m * c) / 2,
    )
