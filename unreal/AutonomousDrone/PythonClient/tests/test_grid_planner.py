from navigation.grid_planner import AltitudeGridPlanner

import numpy as np


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
