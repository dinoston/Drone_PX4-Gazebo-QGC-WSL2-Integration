"""Square-pattern waypoint generation."""

from __future__ import annotations


def build_square_path(
    center_x: float,
    center_y: float,
    altitude_m: float,
    side_m: float = 20.0,
) -> list[tuple[float, float, float]]:
    half = side_m / 2.0
    return [
        (center_x - half, center_y - half, altitude_m),
        (center_x + half, center_y - half, altitude_m),
        (center_x + half, center_y + half, altitude_m),
        (center_x - half, center_y + half, altitude_m),
        (center_x - half, center_y - half, altitude_m),
    ]
