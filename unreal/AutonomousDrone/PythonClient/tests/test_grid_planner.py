from navigation.grid_planner import AltitudeGridPlanner
from navigation.grid_planner import PlannerConfig

import numpy as np
import pytest


def test_direct_path_without_obstacles() -> None:
    planner = AltitudeGridPlanner()
    path = planner.plan((0.0, 0.0), (10.0, 0.0), 5.0, 5.0)
    assert path[-1] == (10.0, 0.0, 5.0)


def test_planner_avoids_wall_horizontally_or_vertically() -> None:
    planner = AltitudeGridPlanner()
    wall = np.array([(5.0, y, -5.0) for y in np.linspace(-4.0, 4.0, 30)], dtype=np.float32)
    planner.set_obstacle_points(wall)
    path = planner.plan((0.0, 0.0), (10.0, 0.0), 5.0, 5.0)
    assert path
    assert any(abs(y) > 4.0 or altitude > 5.0 for _x, y, altitude in path)


def test_planner_can_choose_higher_layer() -> None:
    planner = AltitudeGridPlanner()
    wall = np.array([(0.0, y, -5.0) for y in np.linspace(-100.0, 100.0, 401)], dtype=np.float32)
    planner.set_obstacle_points(wall)
    path = planner.plan((-10.0, 0.0), (10.0, 0.0), 5.0, 5.0)
    assert max(altitude for _x, _y, altitude in path) > 5.0
    assert path[0][0:2] == (-10.0, 0.0)
    assert path[-1] == (10.0, 0.0, 5.0)


def test_planner_keeps_climbing_when_lidar_sees_the_wall_again() -> None:
    planner = AltitudeGridPlanner()
    wall_layers = np.array(
        [
            (0.0, y, -altitude)
            for altitude in (5.0, 7.0, 9.0)
            for y in np.linspace(-100.0, 100.0, 401)
        ],
        dtype=np.float32,
    )
    planner.set_obstacle_points(wall_layers)
    path = planner.plan((-10.0, 0.0), (10.0, 0.0), 5.0, 5.0)
    assert path[0] == (-10.0, 0.0, 11.0)
    assert path[-1] == (10.0, 0.0, 5.0)


def test_replan_does_not_descend_before_clearing_an_obstacle() -> None:
    planner = AltitudeGridPlanner()
    wall = np.array(
        [(0.0, y, -7.0) for y in np.linspace(-100.0, 100.0, 401)],
        dtype=np.float32,
    )
    planner.set_obstacle_points(wall)
    path = planner.plan((-10.0, 0.0), (10.0, 0.0), 5.0, 7.0)
    assert path[0] == (-10.0, 0.0, 9.0)
    assert all(altitude >= 7.0 for _x, _y, altitude in path[:-1])
    assert path[-1] == (10.0, 0.0, 5.0)


def test_replan_respects_the_absolute_climb_limit() -> None:
    planner = AltitudeGridPlanner(PlannerConfig(max_extra_altitude_m=8.0))
    wall = np.array(
        [(0.0, y, -13.0) for y in np.linspace(-100.0, 100.0, 401)],
        dtype=np.float32,
    )
    planner.set_obstacle_points(wall)
    with pytest.raises(RuntimeError):
        planner.plan((-10.0, 0.0), (10.0, 0.0), 5.0, 13.0)


def test_ceiling_cap_forces_a_lower_cruise_layer() -> None:
    planner = AltitudeGridPlanner(PlannerConfig(max_extra_altitude_m=8.0))
    path = planner.plan(
        (-10.0, 0.0),
        (10.0, 0.0),
        5.0,
        13.0,
        max_altitude_m=10.0,
    )
    assert path[0] == (-10.0, 0.0, 10.0)
    assert max(altitude for _x, _y, altitude in path) <= 10.0
    assert path[-1] == (10.0, 0.0, 5.0)
