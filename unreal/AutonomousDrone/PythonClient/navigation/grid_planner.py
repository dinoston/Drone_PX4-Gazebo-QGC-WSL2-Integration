"""Altitude-aware grid A* planner for an AirSim local NED world.

AirSim 로컬 NED 좌표계를 위한 고도 인식 격자 A* 경로 계획기입니다.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from math import ceil, hypot
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class PlannerConfig:
    half_extent_m: float = 100.0
    resolution_m: float = 1.0
    drone_radius_m: float = 1.5
    vertical_clearance_m: float = 0.8
    altitude_step_m: float = 2.0
    max_extra_altitude_m: float = 12.0


class AltitudeGridPlanner:
    """Plan in XY and optionally try higher layers when a route is blocked.

    XY 평면에서 경로를 계획하고, 설정된 경우 막힌 경로의 상위 고도도 탐색합니다.
    """

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        self._points = np.empty((0, 3), dtype=np.float32)

    @property
    def obstacle_points(self) -> np.ndarray:
        return self._points.copy()

    def set_obstacle_points(self, points_ned: np.ndarray) -> None:
        points = np.asarray(points_ned, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            self._points = np.empty((0, 3), dtype=np.float32)
            return
        finite = np.all(np.isfinite(points), axis=1)
        in_map = (
            (np.abs(points[:, 0]) <= self.config.half_extent_m)
            & (np.abs(points[:, 1]) <= self.config.half_extent_m)
        )
        self._points = points[finite & in_map]

    def plan(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        requested_altitude_m: float,
        current_altitude_m: float,
        max_altitude_m: float | None = None,
    ) -> list[tuple[float, float, float]]:
        self._validate_xy(start_xy)
        self._validate_xy(goal_xy)
        # Once avoidance has climbed above the requested altitude, do not plan
        # a descent until the goal. This prevents repeated down/up oscillation.
        # 회피 중 요청 고도보다 높아졌다면 목적지 전에는 하강 경로를 만들지
        # 않습니다. 장애물 앞에서 상하로 반복 진동하는 현상을 방지합니다.
        base = max(
            1.0,
            float(requested_altitude_m),
            float(current_altitude_m),
        )
        # The climb limit is relative to the requested mission altitude, not
        # the latest replan altitude, so repeated scans cannot climb forever.
        # 상승 제한은 최근 재탐색 고도가 아니라 사용자가 지정한 임무 고도를
        # 기준으로 계산하여 반복 감지 때 무한히 상승하지 않도록 합니다.
        configured_maximum = (
            float(requested_altitude_m) + self.config.max_extra_altitude_m
        )
        if max_altitude_m is not None:
            # A ceiling collision establishes a temporary mission altitude cap.
            # 천장 충돌이 발생하면 현재 임무에 임시 최대 고도를 설정합니다.
            configured_maximum = min(
                configured_maximum,
                max(1.0, float(max_altitude_m)),
            )
            base = min(base, configured_maximum)
        maximum_altitude = max(base, configured_maximum)
        steps = max(
            0,
            int((maximum_altitude - base) / self.config.altitude_step_m),
        )

        best: tuple[float, list[tuple[int, int]], float] | None = None
        for index in range(steps + 1):
            altitude = base + index * self.config.altitude_step_m
            blocked = self._blocked_cells(altitude)
            cells = self._astar(self._to_cell(start_xy), self._to_cell(goal_xy), blocked)
            if not cells:
                continue
            cells = self._simplify(cells, blocked)
            horizontal = sum(
                hypot(b[0] - a[0], b[1] - a[1]) * self.config.resolution_m
                for a, b in zip(cells, cells[1:])
            )
            cost = horizontal + abs(altitude - current_altitude_m) * 1.7
            if best is None or cost < best[0]:
                best = (cost, cells, altitude)

        if best is None:
            raise RuntimeError("현재 장애물 지도에서 목적지까지 안전한 경로를 찾지 못했습니다.")

        _, cells, cruise_altitude = best
        path: list[tuple[float, float, float]] = []
        # When a higher layer is selected, climb in place before moving toward
        # the obstacle. A diagonal climb can still touch a tall facade.
        # 더 높은 고도층을 선택하면 장애물 쪽으로 이동하기 전에 제자리에서
        # 먼저 상승합니다. 대각선 상승은 높은 외벽에 계속 닿을 수 있습니다.
        if abs(cruise_altitude - current_altitude_m) > 0.25:
            path.append(
                (float(start_xy[0]), float(start_xy[1]), cruise_altitude)
            )
        path.extend(
            (*self._from_cell(cell), cruise_altitude) for cell in cells[1:]
        )
        # Return to the requested mission altitude only after reaching the goal.
        # 목적지에 도착한 뒤에만 사용자가 지정한 임무 고도로 복귀합니다.
        if cruise_altitude != requested_altitude_m:
            path.append((float(goal_xy[0]), float(goal_xy[1]), float(requested_altitude_m)))
        if not path:
            path.append((float(goal_xy[0]), float(goal_xy[1]), float(requested_altitude_m)))
        return path

    def route_blocked(self, path: Iterable[tuple[float, float, float]]) -> bool:
        points = list(path)
        if not points:
            return False
        previous = points[0]
        for point in points:
            blocked = self._blocked_cells(point[2])
            if not self._line_clear(
                self._to_cell((previous[0], previous[1])),
                self._to_cell((point[0], point[1])),
                blocked,
            ):
                return True
            previous = point
        return False

    def _blocked_cells(self, altitude_m: float) -> set[tuple[int, int]]:
        if not self._points.size:
            return set()
        obstacle_altitudes = -self._points[:, 2]
        layer = np.abs(obstacle_altitudes - altitude_m) <= self.config.vertical_clearance_m
        layer_points = self._points[layer, :2]
        blocked: set[tuple[int, int]] = set()
        inflation = int(ceil(self.config.drone_radius_m / self.config.resolution_m))
        for xy in layer_points:
            center = self._to_cell((float(xy[0]), float(xy[1])))
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    if hypot(dx, dy) * self.config.resolution_m <= self.config.drone_radius_m:
                        candidate = (center[0] + dx, center[1] + dy)
                        if self._inside(candidate):
                            blocked.add(candidate)
        return blocked

    def _astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        blocked: set[tuple[int, int]],
    ) -> list[tuple[int, int]]:
        blocked = blocked - {start, goal}
        frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start: 0.0}
        moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                result = [goal]
                while result[-1] != start:
                    result.append(came_from[result[-1]])
                result.reverse()
                return result
            for dx, dy, step in moves:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._inside(nxt) or nxt in blocked:
                    continue
                new_cost = cost[current] + step
                if new_cost >= cost.get(nxt, float("inf")):
                    continue
                cost[nxt] = new_cost
                came_from[nxt] = current
                heuristic = hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
        return []

    def _simplify(
        self, cells: list[tuple[int, int]], blocked: set[tuple[int, int]]
    ) -> list[tuple[int, int]]:
        if len(cells) < 3:
            return cells
        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            furthest = anchor + 1
            for candidate in range(anchor + 2, len(cells)):
                if self._line_clear(cells[anchor], cells[candidate], blocked):
                    furthest = candidate
                else:
                    break
            result.append(cells[furthest])
            anchor = furthest
        return result

    @staticmethod
    def _line_clear(a: tuple[int, int], b: tuple[int, int], blocked: set[tuple[int, int]]) -> bool:
        count = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
        for index in range(count + 1):
            ratio = index / count
            cell = (round(a[0] + (b[0] - a[0]) * ratio), round(a[1] + (b[1] - a[1]) * ratio))
            if cell in blocked:
                return False
        return True

    def _to_cell(self, xy: tuple[float, float]) -> tuple[int, int]:
        half = self.config.half_extent_m
        resolution = self.config.resolution_m
        return (round((xy[0] + half) / resolution), round((xy[1] + half) / resolution))

    def _from_cell(self, cell: tuple[int, int]) -> tuple[float, float]:
        half = self.config.half_extent_m
        resolution = self.config.resolution_m
        return (cell[0] * resolution - half, cell[1] * resolution - half)

    def _inside(self, cell: tuple[int, int]) -> bool:
        maximum = round(2 * self.config.half_extent_m / self.config.resolution_m)
        return 0 <= cell[0] <= maximum and 0 <= cell[1] <= maximum

    def _validate_xy(self, xy: tuple[float, float]) -> None:
        if abs(xy[0]) > self.config.half_extent_m or abs(xy[1]) > self.config.half_extent_m:
            raise ValueError(f"미니맵 범위는 ±{self.config.half_extent_m:.0f}m입니다.")
