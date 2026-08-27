"""Simple safe-altitude A-to-B delivery route generation."""

from __future__ import annotations


def build_delivery_path(
    start_x: float,
    start_y: float,
    destination_x: float,
    destination_y: float,
    cruise_altitude_m: float,
) -> list[tuple[float, float, float]]:
    return [
        (start_x, start_y, cruise_altitude_m),
        (destination_x, destination_y, cruise_altitude_m),
    ]
