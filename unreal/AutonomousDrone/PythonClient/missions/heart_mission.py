"""Heart-pattern waypoint generation."""

from __future__ import annotations

import math


def build_heart_path(
    center_x: float,
    center_y: float,
    altitude_m: float,
    width_m: float = 30.0,
    samples: int = 80,
) -> list[tuple[float, float, float]]:
    raw: list[tuple[float, float]] = []
    for index in range(samples + 1):
        t = 2.0 * math.pi * index / samples
        x = 16.0 * math.sin(t) ** 3
        y = 13.0 * math.cos(t) - 5.0 * math.cos(2.0 * t) - 2.0 * math.cos(3.0 * t) - math.cos(4.0 * t)
        raw.append((x, y))

    scale = width_m / 32.0
    # AirSim X is forward/north and Y is right/east. Rotate the conventional
    # heart so it is easy to inspect from a top-down view.
    return [
        (center_x - y * scale, center_y + x * scale, altitude_m)
        for x, y in raw
    ]
