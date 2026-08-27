"""Coordinate conversions used by the mission-control application."""

from __future__ import annotations

import numpy as np


def altitude_to_ned_z(altitude_m: float) -> float:
    """Convert a positive-up UI altitude to AirSim's positive-down NED Z."""
    return -abs(float(altitude_m))


def ned_points_to_display(points: np.ndarray) -> np.ndarray:
    """Convert NED point-cloud coordinates to a Z-up display coordinate system."""
    converted = np.asarray(points, dtype=np.float32).copy()
    if converted.size:
        converted[:, 2] *= -1.0
    return converted


def radians_to_degrees(value: float) -> float:
    return float(np.degrees(value))
