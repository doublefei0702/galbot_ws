#!/usr/bin/env python3
"""Run a real Galbot physics route around the east obstacle without rendering."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.replicator.mobility_gen.impl.types import Pose2d  # noqa: E402
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import new_world  # noqa: E402
from galbot.mobility_gen.auto_navigation import astar, load_ros_grid  # noqa: E402
from galbot.mobility_gen.local_workspace import load_workspace, WorkspaceMotionGuard  # noqa: E402
from galbot.mobility_gen.robots import GalbotS1Robot  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--map", dest="map_yaml", type=Path, required=True)
    parser.add_argument("--workspace-config", type=Path, required=True)
    parser.add_argument("--start", type=float, nargs=2, default=(23.0, -7.0))
    parser.add_argument("--goal", type=float, nargs=2, default=(28.0, -7.0))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-seconds", type=float, default=45.0)
    parser.add_argument("--routing-margin", type=float, default=0.20,
                        help="extra planner clearance beyond the robot safety radius, in metres")
    parser.add_argument("--via", type=float, nargs=2, action="append", default=[], metavar=("X", "Y"),
                        help="optional clearance waypoints for a physically smoother detour")
    args, _ = parser.parse_known_args()
    workspace = load_workspace(args.workspace_config)
    grid = load_ros_grid(args.map_yaml)
    safe_grid = grid.inflated(workspace.clearance_m)
    route_grid = grid.inflated(workspace.clearance_m + args.routing_margin)
    start, goal = tuple(args.start), tuple(args.goal)
    if workspace.point_reason(*start) or workspace.point_reason(*goal):
        raise ValueError("route endpoint violates workspace footprint")
    planned = astar(route_grid, start, goal)
    if planned is None or not route_grid.path_is_free(planned):
        raise ValueError("expanded route cannot be planned through inflated map")
    path = [start, *(tuple(v) for v in args.via), goal] if args.via else planned
    if not safe_grid.path_is_free(path) or any(workspace.point_reason(*p) for p in path):
        raise ValueError("selected route violates map or workspace footprint")
    open_stage(str(args.scene.resolve()))
    world = new_world(physics_dt=GalbotS1Robot.physics_dt, stage_units_in_meters=1.0)
    robot = GalbotS1Robot.build("/World/robot")
    world.reset()
    robot.set_pose_2d(Pose2d(x=start[0], y=start[1], theta=0.0))
    if workspace.initialize_held_joints:
        if not robot.initialize():
            raise RuntimeError("robot articulation was not initialized")
        robot.articulation_view.set_joint_positions(
            robot._hold_targets[None, :], joint_indices=robot._idx_hold)
    robot.motion_guard = WorkspaceMotionGuard(workspace, safe_grid)
    dt = float(world.get_physics_dt())
    for _ in range(420):
        robot.action.set_value(np.zeros(3))
        robot.write_action(dt)
        world.step(render=False)
    way_index = 0
    trajectory = []
    stop_reason = "time_limit"
    max_steps = math.ceil(args.max_seconds / dt)
    for step in range(max_steps):
        pose = robot.get_pose_2d()
        xy = (float(pose.x), float(pose.y))
        if math.dist(xy, goal) <= 0.22:
            stop_reason = "goal_reached"
            break
        while way_index < len(path) - 1 and math.dist(xy, path[way_index + 1]) < 0.27:
            way_index += 1
        waypoint = path[min(way_index + 1, len(path) - 1)]
        desired = math.atan2(waypoint[1] - xy[1], waypoint[0] - xy[0])
        error = (desired - pose.theta + math.pi) % (2 * math.pi) - math.pi
        speed = 0.28 * max(0.0, math.cos(error)) if abs(error) < 1.5 else 0.0
        command = np.array([speed, 0.0, float(np.clip(1.8 * error, -1.0, 1.0))])
        robot.action.set_value(command)
        robot.write_action(dt)
        if robot.last_motion_block_reason:
            stop_reason = robot.last_motion_block_reason
            break
        world.step(render=False)
        if step % 20 == 0:
            root_position = np.asarray(robot.articulation_view.get_world_poses()[0]).reshape(-1, 3)[0]
            trajectory.append([round((step + 1) * dt, 3), float(pose.x), float(pose.y),
                               float(root_position[2])])
    end = robot.get_pose_2d()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "status": "PASS" if stop_reason == "goal_reached" else "FAIL",
        "stop_reason": stop_reason,
        "start_world": start,
        "goal_world": goal,
        "end_world": [float(end.x), float(end.y), float(end.theta)],
        "goal_error_m": math.dist((float(end.x), float(end.y)), goal),
        "straight_path_free": safe_grid.segment_is_free(start, goal),
        "routing_clearance_m": workspace.clearance_m + args.routing_margin,
        "via_world": args.via,
        "planned_path_world": path,
        "planned_length_m": sum(math.dist(a, b) for a, b in zip(path, path[1:])),
        "trajectory_time_xy_root_z": trajectory,
        "trajectory_all_safe": all(
            workspace.point_reason(item[1], item[2]) is None and
            safe_grid.is_free_world(item[1], item[2]) for item in trajectory),
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "trajectory_time_xy_root_z"}, indent=2))
    app.close()
    return 0 if report["status"] == "PASS" and report["trajectory_all_safe"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
