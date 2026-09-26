"""Small deterministic 2-D planner used by the headless data collector.

The planner deliberately stays independent from Isaac Sim.  It consumes the
same ROS occupancy-map YAML used by MobilityGen, treats unknown cells as
blocked, inflates obstacles by the configured robot footprint, and checks the
continuous swept segment between grid cells.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image
import yaml


@dataclass(frozen=True)
class GridMap:
    free: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def height(self) -> int:
        return int(self.free.shape[0])

    @property
    def width(self) -> int:
        return int(self.free.shape[1])

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        dx, dy = float(x) - self.origin_x, float(y) - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((c * dx + s * dy) / self.resolution),
                math.floor((-s * dx + c * dy) / self.resolution))

    def grid_to_world(self, col: int, row: int) -> tuple[float, float]:
        lx = (float(col) + 0.5) * self.resolution
        ly = (float(row) + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c * lx - s * ly,
                self.origin_y + s * lx + c * ly)

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def is_free_cell(self, col: int, row: int) -> bool:
        return self.in_bounds(col, row) and bool(self.free[row, col])

    def is_free_world(self, x: float, y: float) -> bool:
        return self.is_free_cell(*self.world_to_grid(x, y))

    def inflated(self, radius_m: float) -> "GridMap":
        """Return a map whose free cells contain the complete circular footprint."""
        radius_px = max(0, int(math.ceil(float(radius_m) / self.resolution)))
        safe = self.free.copy()
        offsets = []
        for dr in range(-radius_px, radius_px + 1):
            for dc in range(-radius_px, radius_px + 1):
                if math.hypot(dc, dr) <= radius_px + 1e-9:
                    offsets.append((dc, dr))
        for dc, dr in offsets:
            shifted = np.zeros_like(self.free)
            src_r0 = max(0, -dr)
            src_r1 = min(self.height, self.height - dr)
            src_c0 = max(0, -dc)
            src_c1 = min(self.width, self.width - dc)
            dst_r0, dst_r1 = src_r0 + dr, src_r1 + dr
            dst_c0, dst_c1 = src_c0 + dc, src_c1 + dc
            shifted[dst_r0:dst_r1, dst_c0:dst_c1] = self.free[src_r0:src_r1, src_c0:src_c1]
            safe &= shifted
        return GridMap(safe, self.resolution, self.origin_x, self.origin_y, self.origin_yaw)

    def segment_is_free(self, start: tuple[float, float], end: tuple[float, float]) -> bool:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(1, int(math.ceil(distance / max(self.resolution * 0.5, 1e-6))))
        for alpha in np.linspace(0.0, 1.0, count + 1):
            x = start[0] + alpha * (end[0] - start[0])
            y = start[1] + alpha * (end[1] - start[1])
            if not self.is_free_world(x, y):
                return False
        return True

    def path_is_free(self, path: Iterable[tuple[float, float]]) -> bool:
        points = list(path)
        return all(self.segment_is_free(a, b) for a, b in zip(points, points[1:])) and all(
            self.is_free_world(*point) for point in points
        )


def load_ros_grid(path: str | Path) -> GridMap:
    path = Path(path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}
    image_path = Path(str(cfg["image"]))
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float32) / 255.0
    probability = pixels if int(cfg.get("negate", 0)) else 1.0 - pixels
    free = probability <= float(cfg.get("free_thresh", 0.196))
    occupied = probability >= float(cfg.get("occupied_thresh", 0.65))
    # ROS maps are stored top-to-bottom while world rows increase upward.
    free = np.flipud(free & ~occupied)
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    if len(origin) != 3:
        raise ValueError(f"{path}: origin must contain [x, y, yaw]")
    return GridMap(free, float(cfg["resolution"]), float(origin[0]),
                   float(origin[1]), float(origin[2]))


def _neighbors(col: int, row: int):
    for dc, dr, cost in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                         (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                         (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0))):
        yield col + dc, row + dr, cost


def astar(grid: GridMap, start_xy: tuple[float, float], goal_xy: tuple[float, float],
          max_nodes: int = 500_000) -> list[tuple[float, float]] | None:
    start = grid.world_to_grid(*start_xy)
    goal = grid.world_to_grid(*goal_xy)
    if not grid.is_free_cell(*start) or not grid.is_free_cell(*goal):
        return None
    frontier = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    cost_so_far = {start: 0.0}
    expanded = 0
    while frontier and expanded < max_nodes:
        _, current = heapq.heappop(frontier)
        expanded += 1
        if current == goal:
            cells = []
            node: tuple[int, int] | None = current
            while node is not None:
                cells.append(node)
                node = came_from[node]
            cells.reverse()
            points = [grid.grid_to_world(col, row) for col, row in cells]
            points[0], points[-1] = start_xy, goal_xy
            return simplify_path(grid, points)
        for nxt_col, nxt_row, step_cost in _neighbors(*current):
            if not grid.is_free_cell(nxt_col, nxt_row):
                continue
            nxt = (nxt_col, nxt_row)
            new_cost = cost_so_far[current] + step_cost
            if new_cost < cost_so_far.get(nxt, float("inf")):
                cost_so_far[nxt] = new_cost
                h = math.hypot(goal[0] - nxt_col, goal[1] - nxt_row)
                heapq.heappush(frontier, (new_cost + h, nxt))
                came_from[nxt] = current
    return None


def simplify_path(grid: GridMap, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    simplified = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        candidate = anchor + 1
        for index in range(anchor + 2, len(points)):
            if grid.segment_is_free(points[anchor], points[index]):
                candidate = index
            else:
                break
        simplified.append(points[candidate])
        anchor = candidate
    return simplified


def sample_reachable_goal(grid: GridMap, safe_grid: GridMap, start_xy: tuple[float, float],
                          rng: np.random.Generator, min_distance_m: float = 1.0,
                          max_attempts: int = 64, path_validator=None
                          ) -> tuple[tuple[float, float], list[tuple[float, float]]] | None:
    if not safe_grid.is_free_world(*start_xy):
        return None
    cells = np.argwhere(safe_grid.free)
    if len(cells) == 0:
        return None
    for index in rng.permutation(len(cells))[:max_attempts]:
        row, col = (int(v) for v in cells[index])
        goal = safe_grid.grid_to_world(col, row)
        if math.hypot(goal[0] - start_xy[0], goal[1] - start_xy[1]) < min_distance_m:
            continue
        path = astar(safe_grid, start_xy, goal)
        if path and grid.path_is_free(path) and (path_validator is None or path_validator(path)):
            return goal, path
    return None
