"""Opt-in local warehouse limits shared by map generation and motion/capture."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path

import yaml


def _inside(x: float, y: float, polygon: tuple[tuple[float, float], ...]) -> bool:
    inside = False
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1]):
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
    return inside


def _edge_distance(x: float, y: float, polygon: tuple[tuple[float, float], ...]) -> float:
    nearest = math.inf
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1]):
        vx, vy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / (vx * vx + vy * vy)))
        nearest = min(nearest, math.hypot(x - ax - t * vx, y - ay - t * vy))
    return nearest


@dataclass(frozen=True)
class LocalWorkspace:
    source: Path
    polygon: tuple[tuple[float, float], ...]
    excludes: tuple[tuple[tuple[float, float], ...], ...]
    footprint_radius_m: float
    safety_margin_m: float
    depth_min_m: float
    depth_max_m: float
    max_invalid_depth_fraction: float
    min_usable_frame_fraction: float
    poor_frame_action: str
    ground_z_m: float
    camera_height_m: float
    camera_height_tolerance_m: float
    initialize_held_joints: bool
    camera_pitch_deg: float | None
    camera_yaw_deg: float
    camera_pitch_tolerance_deg: float
    camera_yaw_tolerance_deg: float
    max_initial_arm_error_rad: float

    @property
    def clearance_m(self) -> float:
        return self.footprint_radius_m + self.safety_margin_m

    def point_reason(self, x: float, y: float, clearance: float | None = None) -> str | None:
        radius = self.clearance_m if clearance is None else float(clearance)
        if not _inside(x, y, self.polygon) or _edge_distance(x, y, self.polygon) <= radius:
            return "workspace_boundary"
        for polygon in self.excludes:
            if _inside(x, y, polygon) or _edge_distance(x, y, polygon) <= radius:
                return "excluded_region"
        return None

    def segment_reason(self, start, end, grid=None, step_m: float = 0.025) -> str | None:
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(1, math.ceil(length / step_m))
        for i in range(count + 1):
            t = i / count
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            reason = self.point_reason(x, y)
            if reason:
                return reason
            if grid is not None and not grid.is_free_world(x, y):
                return "occupied_or_unknown_map_cell"
        return None

    def mask_grid_for_footprint(self, grid):
        """Apply the same polygon and footprint clearance used by motion guards."""
        import numpy as np

        free = grid.free.copy()
        for row, col in np.argwhere(free):
            x, y = grid.grid_to_world(int(col), int(row))
            if self.point_reason(x, y):
                free[row, col] = False
        return replace(grid, free=free)


def load_workspace(path: str | Path) -> LocalWorkspace:
    source = Path(path).expanduser().resolve()
    cfg = yaml.safe_load(source.read_text(encoding="utf-8"))
    if cfg.get("schema_version") != 1:
        raise ValueError("unsupported workspace schema")

    def polygon(points):
        result = tuple((float(p[0]), float(p[1])) for p in points)
        if len(result) < 3 or not all(math.isfinite(v) for p in result for v in p):
            raise ValueError("workspace polygons need at least three finite points")
        return result

    action = str(cfg.get("poor_frame_action", "mark"))
    if action not in ("mark", "skip"):
        raise ValueError("poor_frame_action must be mark or skip")
    result = LocalWorkspace(
        source, polygon(cfg["workspace_polygon_world"]),
        tuple(polygon(p) for p in cfg.get("exclude_polygons_world", [])),
        float(cfg["footprint_radius_m"]), float(cfg["safety_margin_m"]),
        float(cfg["depth_min_m"]), float(cfg["depth_max_m"]),
        float(cfg["max_invalid_depth_fraction"]),
        float(cfg.get("min_usable_frame_fraction", 0.0)), action,
        float(cfg["ground_z_m"]), float(cfg["camera_height_m"]),
        float(cfg.get("camera_height_tolerance_m", 0.0)),
        bool(cfg.get("initialize_held_joints", False)),
        float(cfg["camera_pitch_deg"]) if cfg.get("camera_pitch_deg") is not None else None,
        float(cfg.get("camera_yaw_deg", 0.0)),
        float(cfg.get("camera_pitch_tolerance_deg", 10.0)),
        float(cfg.get("camera_yaw_tolerance_deg", 10.0)),
        float(cfg.get("max_initial_arm_error_rad", 0.25)),
    )
    if (result.footprint_radius_m <= 0 or result.safety_margin_m < 0 or
            result.depth_min_m <= 0 or result.depth_max_m <= result.depth_min_m or
            not 0 <= result.max_invalid_depth_fraction <= 1 or
            not 0 <= result.min_usable_frame_fraction <= 1 or
            result.camera_height_m <= 0 or result.camera_height_tolerance_m < 0 or
            (result.camera_pitch_deg is not None and not math.isfinite(result.camera_pitch_deg)) or
            not math.isfinite(result.camera_yaw_deg) or
            not 0 <= result.camera_pitch_tolerance_deg <= 180 or
            not 0 <= result.camera_yaw_tolerance_deg <= 180 or
            not 0 < result.max_initial_arm_error_rad <= math.pi):
        raise ValueError("invalid workspace clearance or depth limits")
    return result


class WorkspaceMotionGuard:
    """Check every intermediate commanded pose before direct pose integration."""

    def __init__(self, workspace: LocalWorkspace, safe_grid):
        self.workspace = workspace
        self.safe_grid = safe_grid

    def __call__(self, pose, command, dt: float) -> str | None:
        import numpy as np
        from galbot.mobility_gen.base_controller import PlanarBaseController

        values = np.asarray(command, dtype=float).reshape(-1)
        linear = math.hypot(float(values[0]), float(values[1])) * dt
        angular = abs(float(values[2])) * dt
        steps = max(1, math.ceil(linear / 0.025), math.ceil(angular / 0.05))
        prev = (float(pose.x), float(pose.y))
        for i in range(1, steps + 1):
            x, y, _ = PlanarBaseController.integrate(
                float(pose.x), float(pose.y), float(pose.theta), values, dt * i / steps)
            reason = self.workspace.segment_reason(prev, (x, y), self.safe_grid)
            if reason:
                return reason
            prev = (x, y)
        return None
