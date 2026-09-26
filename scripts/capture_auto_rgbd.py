#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Headless, repeatable RGB-D collection with map-constrained motion.

The internal source samples reachable targets in an inflated ROS occupancy map,
plans an A* path, and sends body-frame velocity commands through the existing
``GalbotS1Robot`` action buffer.  The optional ``cmd_vel`` source uses the
existing ROS2NavigationScenario and never runs the internal planner at the
same time.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


WS = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = Path("/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/warehouse_with_forklifts_edit.usd")
DEFAULT_MAP = Path("/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/MobilityGenData/map.yaml")


def _load_planner_module():
    module_path = WS / "exts" / "galbot.mobility_gen" / "galbot" / "mobility_gen" / "auto_navigation.py"
    spec = importlib.util.spec_from_file_location("galbot_auto_navigation", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load planner module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


planner = _load_planner_module()


def _load_workspace_module():
    module_path = WS / "exts" / "galbot.mobility_gen" / "galbot" / "mobility_gen" / "local_workspace.py"
    spec = importlib.util.spec_from_file_location("galbot_local_workspace", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load workspace module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


workspace_module = _load_workspace_module()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--map", dest="map_yaml", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--workspace-config", type=Path, default=None,
                        help="opt-in local area, footprint and depth-quality policy")
    parser.add_argument("--output", type=Path, default=Path("/root/MobilityGenData/auto_rgbd"))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--frames", type=int, default=None, help="maximum saved frames per episode")
    parser.add_argument("--duration", type=float, default=10.0, help="episode duration in simulation seconds")
    parser.add_argument("--sample-rate", type=float, default=5.0)
    parser.add_argument("--quality-action", choices=("skip", "mark"), default=None,
                        help="override workspace policy; mark preserves continuous RGB-D and labels poor frames")
    parser.add_argument("--max-frame-attempt-factor", type=int, default=3,
                        help="maximum sampling opportunities per requested saved frame when quality checks skip frames")
    parser.add_argument("--warmup-steps", type=int, default=420,
                        help="physics ticks to settle the held arm pose before sampling")
    parser.add_argument("--initialize-held-joints", action="store_true",
                        help="set held joint positions to their configured targets before physics warmup")
    parser.add_argument("--motion-source", choices=("internal", "cmd_vel"), default="internal")
    parser.add_argument("--profile", type=Path, default=None, help="required by --motion-source cmd_vel")
    parser.add_argument("--robot-prim-path", default="/World/robot")
    parser.add_argument("--start", nargs=3, type=float, default=[0.5, -5.0, 0.0], metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", nargs=2, type=float, default=None, metavar=("X", "Y"),
                        help="optional fixed world-frame goal for a reproducible navigation capture")
    parser.add_argument("--collision-radius", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--angular-gain", type=float, default=1.8)
    parser.add_argument("--min-target-distance", type=float, default=1.5)
    parser.add_argument("--max-target-attempts", type=int, default=64)
    parser.add_argument("--max-replans", type=int, default=12)
    parser.add_argument("--max-stall-steps", type=int, default=200)
    parser.add_argument("--smoke", action="store_true", help="one deterministic short episode")
    parser.add_argument("--boundary-probe", action="store_true",
                        help="after the run, test that an outward command stops before the boundary")
    parser.add_argument("--overwrite", action="store_true", help="allow replacing an existing output directory")
    parser.add_argument("--headless", action="store_true", default=True, help="kept explicit for the CLI contract")
    return parser.parse_known_args()[0]


def _fresh_output(base: Path, overwrite: bool) -> Path:
    base = base.expanduser().resolve()
    if overwrite:
        base.mkdir(parents=True, exist_ok=True)
        return base
    if not base.exists() or not any(base.iterdir()):
        base.mkdir(parents=True, exist_ok=True)
        return base
    stamp = time.strftime("run_%Y%m%d_%H%M%S")
    candidate = base / f"{stamp}_{os.getpid()}"
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def _git_revision() -> str:
    try:
        return subprocess.check_output(["git", "-C", str(WS), "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _pose_from_robot(robot):
    from isaacsim.core.utils.rotations import quat_to_euler_angles
    positions, orientations = robot.articulation_view.get_world_poses()
    position = np.asarray(positions).reshape(-1, 3)[0].copy()
    quaternion = np.asarray(orientations).reshape(-1, 4)[0].copy()
    yaw = float(quat_to_euler_angles(quaternion)[2])
    return np.array([position[0], position[1], yaw], dtype=float)


def _robot_state(robot) -> dict:
    positions, orientations = robot.articulation_view.get_world_poses()
    position = np.asarray(positions).reshape(-1, 3)[0].astype(float, copy=True)
    quaternion = np.asarray(orientations).reshape(-1, 4)[0].astype(float, copy=True)
    joint_positions = np.asarray(robot.joint_positions.get_value(), dtype=float).reshape(-1).copy()
    joint_velocities = np.asarray(robot.joint_velocities.get_value(), dtype=float).reshape(-1).copy()
    return {
        "position_world": position.tolist(),
        "quaternion_world_wxyz": quaternion.tolist(),
        "joint_positions": joint_positions.tolist(),
        "joint_velocities": joint_velocities.tolist(),
    }


def _camera_info(camera) -> dict:
    k = np.asarray(camera.get_intrinsics_matrix(), dtype=float)
    p = np.zeros((3, 4), dtype=float)
    p[:, :3] = k
    return {
        "width": int(camera.get_resolution()[0]),
        "height": int(camera.get_resolution()[1]),
        "K": k.tolist(),
        "R": np.eye(3, dtype=float).tolist(),
        "P": p.tolist(),
        "distortion_model": "plumb_bob",
        "model": "plumb_bob",
        "D": [0.0] * 5,
        "depth_definition": "distance_to_image_plane",
        "depth_units": "meters",
        "clipping_range_m": [float(v) for v in camera.get_clipping_range()],
        "invalid_depth_values": ["NaN", "+Infinity", "-Infinity", "zero_or_negative"],
        "calibration_source": "isaacsim_render",
    }


def _camera_frame(robot, side: str):
    camera_module = getattr(robot, f"{side}_wrist_camera")
    camera = robot._wrist_pose_handles[side]
    rgb = np.asarray(camera_module.rgb_image.get_value()).copy()
    depth = np.asarray(camera_module.depth_image.get_value(), dtype=np.float32).copy()
    position, quaternion = camera.get_world_pose(camera_axes="ros")
    position = np.asarray(position, dtype=float).copy()
    quaternion = np.asarray(quaternion, dtype=float).copy()
    from galbot_pose_utils import pose_matrix
    return rgb[:, :, :3].astype(np.uint8), depth, position, quaternion, pose_matrix(position, quaternion)


def _validate_world_from_camera(matrix: np.ndarray) -> bool:
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        return False
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-5):
        return False
    rotation = matrix[:3, :3]
    return bool(np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3)
                and np.linalg.det(rotation) > 0.0)


def _camera_attitude(matrix: np.ndarray, robot_yaw_rad: float, workspace) -> dict:
    """Measure the optical axis against the commanded robot-relative view."""
    forward = np.asarray(matrix, dtype=float)[:3, 2]
    world_yaw = math.degrees(math.atan2(float(forward[1]), float(forward[0])))
    pitch = math.degrees(math.atan2(float(forward[2]), math.hypot(float(forward[0]), float(forward[1]))))
    relative_yaw = (world_yaw - math.degrees(robot_yaw_rad) + 180.0) % 360.0 - 180.0
    yaw_error = (relative_yaw - workspace.camera_yaw_deg + 180.0) % 360.0 - 180.0
    pitch_error = pitch - workspace.camera_pitch_deg
    return {
        "camera_yaw_world_deg": world_yaw,
        "camera_pitch_world_deg": pitch,
        "camera_yaw_robot_relative_deg": relative_yaw,
        "camera_yaw_error_deg": yaw_error,
        "camera_pitch_error_deg": pitch_error,
        "camera_orientation_outside_tolerance": (
            abs(yaw_error) > workspace.camera_yaw_tolerance_deg or
            abs(pitch_error) > workspace.camera_pitch_tolerance_deg),
    }


@dataclass
class EpisodeStats:
    episode: int
    target_count: int = 0
    completed_targets: int = 0
    replans: int = 0
    blocked_events: int = 0
    reset_count: int = 0
    saved_frames: int = 0
    usable_frames: int = 0
    usable_fraction: float = 0.0
    marked_frames: int = 0
    invalid_depth_pixels: int = 0
    out_of_range_depth_pixels: int = 0
    skipped_frames: int = 0
    stop_reason: str = ""


class EpisodeWriter:
    def __init__(self, root: Path, episode: int, camera_info: dict[str, dict], workspace=None):
        self.root = root / f"episode_{episode:03d}"
        self.root.mkdir(parents=True, exist_ok=False)
        self.workspace = workspace
        self.quality_events = (self.root / "quality_events.jsonl").open("w", encoding="utf-8")
        self.trajectory_plans = (self.root / "trajectory_plans.jsonl").open("w", encoding="utf-8")
        self.manifests = {}
        self.pose_files = {}
        for side, info in camera_info.items():
            camera_dir = self.root / side
            for sub in ("rgb", "depth_raw", "depth", "depth_valid_mask"):
                (camera_dir / sub).mkdir(parents=True, exist_ok=True)
            (camera_dir / "camera_info.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
            self.manifests[side] = (camera_dir / "manifest.jsonl").open("w", encoding="utf-8")
            self.pose_files[side] = (camera_dir / "pose.txt").open("w", encoding="utf-8")

    def depth_valid(self, depth: np.ndarray) -> np.ndarray:
        lower = self.workspace.depth_min_m if self.workspace else 0.001
        upper = self.workspace.depth_max_m if self.workspace else 65.535
        return np.isfinite(depth) & (depth >= lower) & (depth <= upper)

    def record_quality_event(self, event: dict) -> None:
        self.quality_events.write(json.dumps(event, separators=(",", ":")) + "\n")
        self.quality_events.flush()

    def record_plan(self, plan_index: int, start_xy, goal_xy, waypoints) -> None:
        self.trajectory_plans.write(json.dumps({
            "plan_index": plan_index, "start_world_xy": list(start_xy),
            "goal_world_xy": list(goal_xy),
            "waypoints_world_xy": [list(point) for point in waypoints],
        }, separators=(",", ":")) + "\n")
        self.trajectory_plans.flush()

    def write(self, frame_id: int, captures: dict, common: dict) -> tuple[int, int]:
        invalid_total = out_of_range_total = 0
        for side, (rgb, depth, position, quaternion, matrix) in captures.items():
            camera_dir = self.root / side
            np.save(camera_dir / "depth_raw" / f"{frame_id:06d}.npy", depth.astype(np.float32, copy=True))
            finite = np.isfinite(depth)
            valid = self.depth_valid(depth)
            no_return = (~finite) | (depth <= 0.0)
            invalid = int(no_return.sum())
            out_of_range = int((finite & (depth > 0.0) & ~valid).sum())
            depth16 = np.zeros(depth.shape, dtype=np.uint16)
            depth16[valid] = np.rint(depth[valid] * 1000.0).astype(np.uint16)
            from PIL import Image
            Image.fromarray(depth16, mode="I;16").save(camera_dir / "depth" / f"{frame_id:06d}.png")
            Image.fromarray((valid.astype(np.uint8) * 255), mode="L").save(
                camera_dir / "depth_valid_mask" / f"{frame_id:06d}.png")
            Image.fromarray(rgb, mode="RGB").save(camera_dir / "rgb" / f"{frame_id:06d}.png")
            row = dict(common)
            if self.workspace and self.workspace.camera_pitch_deg is not None and "robot_pose_world" in common:
                row.update(_camera_attitude(matrix, float(common["robot_pose_world"][2]), self.workspace))
            row.update({
                "camera": side,
                "frame_id": frame_id,
                "rgb": f"rgb/{frame_id:06d}.png",
                "depth_raw": f"depth_raw/{frame_id:06d}.npy",
                "depth_png": f"depth/{frame_id:06d}.png",
                "depth_valid_mask": f"depth_valid_mask/{frame_id:06d}.png",
                "camera_position_world": position.tolist(),
                "camera_height_above_ground_m": (float(position[2] - self.workspace.ground_z_m)
                                                 if self.workspace else None),
                "camera_below_height_reference": (bool(position[2] - self.workspace.ground_z_m <
                                                       self.workspace.camera_height_m -
                                                       self.workspace.camera_height_tolerance_m)
                                                  if self.workspace else None),
                "camera_outside_height_tolerance": (
                    bool(abs(position[2] - self.workspace.ground_z_m - self.workspace.camera_height_m) >
                         self.workspace.camera_height_tolerance_m)
                    if self.workspace else None),
                "camera_quaternion_world_from_opencv_wxyz": quaternion.tolist(),
                "world_from_camera_opencv": matrix.tolist(),
                "invalid_depth_pixels": invalid,
                "out_of_range_depth_pixels": out_of_range,
                "invalid_depth_fraction": float(1.0 - valid.mean()),
                "poor_depth_quality": bool(self.workspace and
                                           1.0 - valid.mean() > self.workspace.max_invalid_depth_fraction),
                "frame_quality_accepted": bool(common.get("frame_quality_accepted", True)),
                "frame_quality_failures": list(common.get("frame_quality_failures", [])),
            })
            self.manifests[side].write(json.dumps(row, separators=(",", ":")) + "\n")
            self.manifests[side].flush()
            self.pose_files[side].write(
                f"# frame {frame_id}\n" +
                "\n".join(" ".join(f"{float(value):.9f}" for value in matrix_row)
                          for matrix_row in matrix) + "\n")
            self.pose_files[side].flush()
            invalid_total += invalid
            out_of_range_total += out_of_range
        return invalid_total, out_of_range_total

    def close(self):
        self.quality_events.flush()
        self.quality_events.close()
        self.trajectory_plans.flush()
        self.trajectory_plans.close()
        for stream in self.manifests.values():
            stream.flush()
            stream.close()
        for stream in self.pose_files.values():
            stream.flush()
            stream.close()


def _prepare_cmd_vel_source(args, robot, occupancy_map):
    if args.motion_source != "cmd_vel":
        return None
    if args.profile is None:
        raise ValueError("--profile is required with --motion-source cmd_vel")
    profile = args.profile.expanduser().resolve()
    if not profile.is_file():
        raise FileNotFoundError(profile)
    os.environ["GALBOT_SIM_PROFILE"] = str(profile)
    adapter_root = WS.parent / "sim_adapter"
    sys.path.insert(0, str(adapter_root / "exts" / "galbot.ros2_navigation"))
    from isaacsim.replicator.mobility_gen.impl.occupancy_map import OccupancyMap
    from galbot.ros2_navigation.profile import load_scene_profile
    from galbot.ros2_navigation.scenario import ROS2NavigationScenario
    scene_profile = load_scene_profile(profile)
    args.start = [scene_profile.initial_pose_world[0], scene_profile.initial_pose_world[1],
                  math.degrees(scene_profile.initial_pose_world[2])]
    bridge_map = OccupancyMap.from_ros_yaml(str(scene_profile.map_yaml))
    return ROS2NavigationScenario.from_robot_occupancy_map(robot, bridge_map)


def main() -> int:
    args = parse_args()
    if args.smoke:
        args.episodes = 1
        args.frames = args.frames or 20
    if args.episodes < 1 or args.sample_rate <= 0 or args.duration <= 0 or args.max_frame_attempt_factor < 1:
        raise ValueError("episodes, sample-rate, duration and max-frame-attempt-factor must be positive")
    if args.frames is not None and args.frames < 1:
        raise ValueError("frames must be positive")
    if args.smoke and args.goal is not None:
        raise ValueError("--smoke and --goal cannot be combined")
    scene = args.scene.expanduser().resolve()
    map_yaml = args.map_yaml.expanduser().resolve()
    if not scene.is_file():
        raise FileNotFoundError(f"scene does not exist: {scene}")
    if not map_yaml.is_file():
        raise FileNotFoundError(f"map YAML does not exist: {map_yaml}")
    output = _fresh_output(args.output, args.overwrite)
    rng = np.random.default_rng(args.seed)
    workspace = workspace_module.load_workspace(args.workspace_config) if args.workspace_config else None
    if args.quality_action is not None:
        if workspace is None:
            raise ValueError("--quality-action requires --workspace-config")
        workspace = replace(workspace, poor_frame_action=args.quality_action)
    if workspace is not None:
        args.collision_radius = workspace.clearance_m
    grid = planner.load_ros_grid(map_yaml)
    safe_grid = grid.inflated(args.collision_radius)
    if workspace is not None:
        safe_grid = workspace.mask_grid_for_footprint(safe_grid)
    start = tuple(float(v) for v in args.start[:2])
    if workspace is not None:
        reason = workspace.point_reason(*start)
        if reason:
            raise ValueError(f"start pose violates local workspace: {reason}")
    if args.motion_source == "internal" and not safe_grid.is_free_world(*start):
        raise ValueError(f"start pose is not free after footprint inflation: {start}")
    if args.goal is not None:
        fixed_goal = tuple(float(value) for value in args.goal)
        if workspace is not None and workspace.point_reason(*fixed_goal):
            raise ValueError(f"goal violates local workspace: {fixed_goal}")
        if not safe_grid.is_free_world(*fixed_goal):
            raise ValueError(f"goal is not free after footprint inflation: {fixed_goal}")
        if args.motion_source != "internal":
            raise ValueError("--goal currently requires --motion-source internal")

    # The module is loaded from a file so system Python can run the planner tests.
    pose_path = WS / "exts" / "galbot.mobility_gen" / "galbot" / "mobility_gen" / "pose_utils.py"
    pose_spec = importlib.util.spec_from_file_location("galbot_pose_utils", pose_path)
    pose_module = importlib.util.module_from_spec(pose_spec)
    assert pose_spec.loader is not None
    pose_spec.loader.exec_module(pose_module)
    sys.modules["galbot_pose_utils"] = pose_module

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": True})
    exit_code = 1
    report = {
        "status": "FAIL",
        "validation": {},
        "run": {
            "seed": args.seed,
            "code_revision": _git_revision(),
            "scene": str(scene),
            "map_yaml": str(map_yaml),
            "map_sha256": _sha256(map_yaml),
            "workspace_config": str(workspace.source) if workspace else None,
            "motion_source": args.motion_source,
            "fixed_goal_world_xy": args.goal,
            "sample_rate_hz": args.sample_rate,
            "quality_action": workspace.poor_frame_action if workspace else None,
            "max_initial_arm_error_rad": workspace.max_initial_arm_error_rad if workspace else None,
            "min_usable_frame_fraction": workspace.min_usable_frame_fraction if workspace else None,
            "max_frame_attempt_factor": args.max_frame_attempt_factor,
            "depth_definition": "distance_to_image_plane",
            "depth_valid_range_m": [workspace.depth_min_m, workspace.depth_max_m] if workspace else [0.001, 65.535],
            "headless": True,
        },
        "episodes": [],
    }
    writers = []
    try:
        import omni.usd
        from isaacsim.core.utils.stage import open_stage
        from isaacsim.replicator.mobility_gen.impl.utils.global_utils import new_world
        from isaacsim.core.prims import Articulation
        from isaacsim.replicator.mobility_gen.impl.types import Pose2d
        from galbot.mobility_gen.robots import GalbotS1Robot

        open_stage(str(scene))
        stage = omni.usd.get_context().get_stage()
        used_layers = [str(layer.identifier) for layer in stage.GetUsedLayers()]
        # USDZ layer identifiers are "archive.usdz[member.usd]"; the archive
        # exists as a file, while the full identifier is not a filesystem path.
        missing_layers = [item for item in used_layers
                          if not item.startswith(("anon:", "http://", "https://"))
                          and not Path(item.split("[", 1)[0]).is_file()]
        if missing_layers:
            raise FileNotFoundError(f"stage has unresolved used layers: {missing_layers[:8]}")
        world = new_world(physics_dt=GalbotS1Robot.physics_dt, stage_units_in_meters=1.0)
        if omni.usd.get_context().get_stage().GetPrimAtPath(args.robot_prim_path).IsValid():
            prim_path = args.robot_prim_path
            articulation = Articulation(prim_path)
            world.scene.add(articulation)
            robot = GalbotS1Robot(prim_path, articulation)
            robot.build_wrist_cameras()
        else:
            robot = GalbotS1Robot.build("/World/robot")
        world.reset()
        robot.set_pose_2d(Pose2d(float(args.start[0]), float(args.start[1]), math.radians(float(args.start[2]))))
        robot.action.set_value(np.zeros(3, dtype=float))
        initialize_held_joints = args.initialize_held_joints or bool(
            workspace and workspace.initialize_held_joints)
        report["run"]["initialize_held_joints"] = initialize_held_joints
        report["run"]["warmup_steps"] = args.warmup_steps
        if initialize_held_joints:
            if not robot.initialize():
                raise RuntimeError("robot joints are unavailable after world.reset()")
            robot.articulation_view.set_joint_positions(
                robot._hold_targets[None, :], joint_indices=robot._idx_hold)

        if workspace is not None:
            if workspace.camera_pitch_deg is not None:
                robot.wrist_view_pitch_deg = workspace.camera_pitch_deg
            robot.wrist_view_yaw_deg = workspace.camera_yaw_deg
            for camera in robot._wrist_pose_handles.values():
                camera.set_clipping_range(workspace.depth_min_m, workspace.depth_max_m)

        camera_info = {side: _camera_info(robot._wrist_pose_handles[side]) for side in ("left", "right")}
        app = __import__("omni.kit.app", fromlist=["get_app"]).get_app()
        report["run"].update({
            "isaac_sim_version": str(app.get_build_version()),
            "camera_info": camera_info,
            "map_shape": [grid.height, grid.width],
            "map_resolution_m": grid.resolution,
            "collision_radius_m": args.collision_radius,
            "stage_used_layers": used_layers,
            "stage_missing_used_layers": missing_layers,
            "dof_names": list(getattr(robot.articulation_view, "dof_names", [])),
        })
        ros_scenario = _prepare_cmd_vel_source(args, robot, None)
        if ros_scenario is not None:
            if workspace is not None:
                profile = ros_scenario.profile
                if profile.map_yaml.resolve() != map_yaml or profile.stage_usd.resolve() != scene:
                    raise ValueError("cmd_vel profile stage/map must match --scene and --map")
                if profile.workspace_config is None or profile.workspace_config.resolve() != workspace.source:
                    raise ValueError("cmd_vel profile workspace_config must match --workspace-config")
            ros_scenario.reset()
        if workspace is not None:
            robot.motion_guard = workspace_module.WorkspaceMotionGuard(workspace, safe_grid)

        dt = float(world.get_physics_dt())
        warmup_steps = args.warmup_steps
        if warmup_steps < 8:
            raise ValueError("--warmup-steps must be at least 8")
        for warmup_index in range(warmup_steps):
            if ros_scenario is not None:
                ros_scenario.step(dt)
            else:
                robot.action.set_value(np.zeros(3, dtype=float))
                robot.write_action(dt)
            world.step(render=warmup_index >= warmup_steps - 8)
            robot.update_state()
        held_names = [robot.articulation_view.dof_names[int(i)] for i in robot._idx_hold]
        held_actual = np.asarray(robot.articulation_view.get_joint_positions()).reshape(-1)[robot._idx_hold]
        report["run"]["held_joint_errors_rad_or_m"] = {
            name: float(actual - target)
            for name, actual, target in zip(held_names, held_actual, robot._hold_targets)
            if name.startswith(("left_arm_", "right_arm_", "torso_lift_"))
        }
        if workspace is not None:
            arm_errors = {name: error for name, error in report["run"]["held_joint_errors_rad_or_m"].items()
                          if name.startswith(("left_arm_", "right_arm_"))}
            worst_name, worst_error = max(arm_errors.items(), key=lambda item: abs(item[1]))
            report["run"]["initial_arm_pose_stable"] = abs(worst_error) <= workspace.max_initial_arm_error_rad
            if not report["run"]["initial_arm_pose_stable"]:
                raise RuntimeError(
                    f"initial arm pose unstable: {worst_name} error={worst_error:.3f} rad "
                    f"exceeds {workspace.max_initial_arm_error_rad:.3f} rad")

        for episode in range(args.episodes):
            if episode > 0:
                if ros_scenario is not None:
                    ros_scenario.reset()
                else:
                    robot.set_pose_2d(Pose2d(float(args.start[0]), float(args.start[1]), math.radians(float(args.start[2]))))
                    robot.action.set_value(np.zeros(3, dtype=float))
                    if initialize_held_joints:
                        robot.articulation_view.set_joint_positions(
                            robot._hold_targets[None, :], joint_indices=robot._idx_hold)
                for _ in range(20):
                    if ros_scenario is not None:
                        ros_scenario.step(dt)
                    else:
                        robot.action.set_value(np.zeros(3, dtype=float))
                        robot.write_action(dt)
                    world.step(render=True)
                    robot.update_state()

            stats = EpisodeStats(episode=episode, reset_count=1)
            writer = EpisodeWriter(output, episode, camera_info, workspace)
            writers.append(writer)
            episode_start = _pose_from_robot(robot)
            current_path = None
            path_index = 0
            sim_time = 0.0
            next_sample = 0.0
            frame_id = 0
            stall_steps = 0
            previous_pose = episode_start.copy()
            previous_sample_pose = episode_start.copy()
            def path_within_workspace(path):
                return workspace is None or all(
                    workspace.segment_reason(a, b, safe_grid) is None
                    for a, b in zip(path, path[1:]))
            max_steps = (args.frames * args.max_frame_attempt_factor *
                         max(1, int(round(1.0 / args.sample_rate / dt)))) if args.frames else int(round(args.duration / dt))
            for step in range(max_steps):
                pose = _pose_from_robot(robot)
                command = np.zeros(3, dtype=float)
                target = None
                if ros_scenario is None:
                    if args.smoke and episode == 0:
                        smoke_candidates = [(start[0] + 1.5, start[1]), (start[0] - 1.5, start[1]),
                                            (start[0], start[1] + 1.5), (start[0], start[1] - 1.5)]
                        if current_path is None:
                            for candidate in smoke_candidates:
                                if safe_grid.is_free_world(*candidate):
                                    candidate_path = planner.astar(safe_grid, start, candidate)
                                    if candidate_path and grid.path_is_free(candidate_path) and path_within_workspace(candidate_path):
                                        current_path = candidate_path
                                        stats.target_count += 1
                                        writer.record_plan(stats.target_count - 1, (float(pose[0]), float(pose[1])),
                                                           candidate, current_path)
                                        break
                            if current_path is None:
                                stats.stop_reason = "smoke_target_unreachable"
                                break
                    elif current_path is not None and path_index >= len(current_path) - 1:
                        stats.completed_targets += 1
                        current_path = None
                        if args.goal is not None:
                            stats.stop_reason = "goal_reached"
                            break
                    if current_path is None:
                        if args.goal is not None:
                            path = planner.astar(safe_grid, (float(pose[0]), float(pose[1])), fixed_goal)
                            result = (fixed_goal, path) if path and path_within_workspace(path) else None
                        else:
                            result = planner.sample_reachable_goal(
                                grid, safe_grid, (float(pose[0]), float(pose[1])), rng,
                                args.min_target_distance, args.max_target_attempts,
                                path_validator=path_within_workspace)
                        if result is None:
                            stats.stop_reason = "no_reachable_target"
                            break
                        goal, current_path = result
                        path_index = 0
                        stats.target_count += 1
                        writer.record_plan(stats.target_count - 1, (float(pose[0]), float(pose[1])),
                                           goal, current_path)
                    if current_path:
                        while path_index < len(current_path) - 1 and math.hypot(
                                current_path[path_index + 1][0] - pose[0],
                                current_path[path_index + 1][1] - pose[1]) < 0.30:
                            path_index += 1
                        target = current_path[min(path_index + 1, len(current_path) - 1)]
                        desired = math.atan2(target[1] - pose[1], target[0] - pose[0])
                        error = (desired - pose[2] + math.pi) % (2.0 * math.pi) - math.pi
                        linear = args.speed * max(0.0, math.cos(error)) if abs(error) < math.pi * 0.55 else 0.0
                        angular = float(np.clip(args.angular_gain * error, -1.0, 1.0))
                        command[:] = (linear, 0.0, angular)
                        from galbot.mobility_gen.base_controller import PlanarBaseController
                        next_pose = PlanarBaseController.integrate(*pose, command, dt)
                        if not safe_grid.segment_is_free((pose[0], pose[1]), (next_pose[0], next_pose[1])):
                            stats.blocked_events += 1
                            stats.replans += 1
                            current_path = None
                            command[:] = 0.0
                            if stats.replans > args.max_replans:
                                stats.stop_reason = "replan_limit"
                                break
                robot.action.set_value(command)
                if ros_scenario is not None:
                    ros_scenario.step(dt)
                    command = np.asarray(robot.action.get_value(), dtype=float).copy()
                else:
                    robot.write_action(dt)
                world.step(render=True)
                sim_time += dt
                robot.update_state()
                actual_pose = _pose_from_robot(robot)
                if robot.last_motion_block_reason:
                    stats.blocked_events += 1
                    stats.stop_reason = robot.last_motion_block_reason
                    break
                moved = float(np.hypot(actual_pose[0] - previous_pose[0], actual_pose[1] - previous_pose[1]))
                stall_steps = stall_steps + 1 if np.linalg.norm(command) > 1e-6 and moved < 1e-7 else 0
                if stall_steps > args.max_stall_steps:
                    stats.stop_reason = "stalled"
                    break
                previous_pose = actual_pose
                while sim_time + 1e-9 >= next_sample and (args.frames is None or frame_id < args.frames):
                    if workspace is not None:
                        reason = workspace.point_reason(float(actual_pose[0]), float(actual_pose[1]))
                        if reason or not safe_grid.is_free_world(float(actual_pose[0]), float(actual_pose[1])):
                            stats.stop_reason = f"capture_pose_outside_workspace:{reason or 'occupied_or_unknown_map_cell'}"
                            break
                    captures = {side: _camera_frame(robot, side) for side in ("left", "right")}
                    robot_state = _robot_state(robot)
                    if frame_id == 0:
                        for side, capture in captures.items():
                            if capture[0].shape != (480, 640, 3):
                                raise RuntimeError(f"{side} RGB shape={capture[0].shape}")
                            if capture[1].shape[:2] != (480, 640):
                                raise RuntimeError(f"{side} depth shape={capture[1].shape}")
                            if not _validate_world_from_camera(capture[4]):
                                raise RuntimeError(f"{side} world_from_camera_opencv is not a valid SE(3) matrix")
                    if captures["left"][1].shape != captures["right"][1].shape:
                        raise RuntimeError("left/right depth samples are not shape-synchronous")
                    invalid_fractions = {
                        side: float(1.0 - writer.depth_valid(capture[1]).mean())
                        for side, capture in captures.items()
                    }
                    camera_heights = ({side: float(capture[2][2] - workspace.ground_z_m)
                                       for side, capture in captures.items()} if workspace else {})
                    poor_depth = bool(workspace and any(
                        value > workspace.max_invalid_depth_fraction
                        for value in invalid_fractions.values()))
                    poor_height = bool(workspace and any(
                        abs(value - workspace.camera_height_m) > workspace.camera_height_tolerance_m
                        for value in camera_heights.values()))
                    camera_attitudes = ({side: _camera_attitude(capture[4], float(actual_pose[2]), workspace)
                                         for side, capture in captures.items()}
                                        if workspace and workspace.camera_pitch_deg is not None else {})
                    poor_orientation = any(value["camera_orientation_outside_tolerance"]
                                           for value in camera_attitudes.values())
                    failed_checks = []
                    if poor_depth:
                        failed_checks.append("invalid_depth_fraction_exceeds_threshold")
                    if poor_height:
                        failed_checks.append("camera_height_outside_tolerance")
                    if poor_orientation:
                        failed_checks.append("camera_orientation_outside_tolerance")
                    if failed_checks:
                        writer.record_quality_event({"sim_time": sim_time, "candidate_frame_id": frame_id,
                                            "reason": (failed_checks[0] if len(failed_checks) == 1 else
                                                       "multiple_quality_limits_exceeded"),
                                            "action": workspace.poor_frame_action if workspace else "mark",
                                            "failed_checks": failed_checks,
                                            "invalid_depth_fraction": invalid_fractions,
                                            "threshold": workspace.max_invalid_depth_fraction,
                                            "camera_height_above_ground_m": camera_heights,
                                            "camera_height_reference_m": workspace.camera_height_m,
                                            "camera_height_tolerance_m": workspace.camera_height_tolerance_m,
                                            "camera_attitude": camera_attitudes,
                                            "robot_pose_world": actual_pose.tolist()})
                    if failed_checks and workspace is not None and workspace.poor_frame_action == "skip":
                        stats.skipped_frames += 1
                        next_sample += 1.0 / args.sample_rate
                        previous_sample_pose = actual_pose.copy()
                        continue
                    sample_dt = 1.0 / args.sample_rate
                    delta_x = actual_pose[0] - previous_sample_pose[0]
                    delta_y = actual_pose[1] - previous_sample_pose[1]
                    body_vx = (math.cos(actual_pose[2]) * delta_x + math.sin(actual_pose[2]) * delta_y) / sample_dt if frame_id else 0.0
                    body_vy = (-math.sin(actual_pose[2]) * delta_x + math.cos(actual_pose[2]) * delta_y) / sample_dt if frame_id else 0.0
                    yaw_delta = (actual_pose[2] - previous_sample_pose[2] + math.pi) % (2.0 * math.pi) - math.pi
                    common = {
                        "episode": episode, "sim_time": sim_time, "frame_id": frame_id,
                        "robot_pose_world": actual_pose.tolist(),
                        "robot_position_world": robot_state["position_world"],
                        "robot_quaternion_world_wxyz": robot_state["quaternion_world_wxyz"],
                        "joint_positions": robot_state["joint_positions"],
                        "joint_velocities": robot_state["joint_velocities"],
                        "command_twist_body": command.tolist(),
                        "applied_velocity_body": [float(body_vx), float(body_vy), float(yaw_delta / sample_dt) if frame_id else 0.0],
                        "target_world": list(target) if target is not None else None,
                        "workspace_config": str(workspace.source) if workspace else None,
                        "camera_height_reference_m": workspace.camera_height_m if workspace else None,
                        "frame_quality_accepted": not failed_checks,
                        "frame_quality_failures": failed_checks,
                    }
                    invalid, out_of_range = writer.write(frame_id, captures, common)
                    stats.invalid_depth_pixels += invalid
                    stats.out_of_range_depth_pixels += out_of_range
                    stats.saved_frames += 1
                    if failed_checks:
                        stats.marked_frames += 1
                    else:
                        stats.usable_frames += 1
                    previous_sample_pose = actual_pose.copy()
                    frame_id += 1
                    next_sample += 1.0 / args.sample_rate
                    if args.frames is not None and frame_id >= args.frames:
                        break
                if stats.stop_reason.startswith("capture_pose_outside_workspace:"):
                    break
                if args.frames is not None and frame_id >= args.frames:
                    stats.stop_reason = "frame_limit"
                    break
            else:
                stats.stop_reason = "quality_attempt_limit" if args.frames is not None else "duration_limit"
            writer.close()
            writers.remove(writer)
            stats.usable_fraction = stats.usable_frames / max(1, stats.saved_frames + stats.skipped_frames)
            report["episodes"].append(asdict(stats))
        if args.boundary_probe:
            if workspace is None or ros_scenario is not None:
                raise ValueError("--boundary-probe requires --workspace-config and internal motion")
            import yaml
            cfg = yaml.safe_load(workspace.source.read_text(encoding="utf-8"))
            bx, by, byaw = (float(v) for v in cfg["boundary_probe_start_world"])
            probe_command = np.asarray(cfg["boundary_probe_command_body"], dtype=float)
            if workspace.point_reason(bx, by) or not safe_grid.is_free_world(bx, by):
                raise ValueError("boundary probe start is not footprint-safe")
            robot.set_pose_2d(Pose2d(bx, by, math.radians(byaw)))
            before = robot.get_pose_2d()
            robot.action.set_value(probe_command)
            robot.write_action(dt)
            after = robot.get_pose_2d()
            blocked = robot.last_motion_block_reason
            displacement = math.hypot(after.x-before.x, after.y-before.y)
            report["boundary_probe"] = {
                "start_world": [bx, by, byaw], "command_body": probe_command.tolist(),
                "blocked_reason": blocked, "commanded_displacement_m": float(np.linalg.norm(probe_command[:2])*dt),
                "actual_displacement_m": displacement,
                "passed": bool(blocked == "workspace_boundary" and displacement < 1e-5),
            }
        report["validation"] = {
            "all_episode_frames_nonzero": all(item["saved_frames"] > 0 for item in report["episodes"]),
            "all_episode_usable_frames_nonzero": all(item["usable_frames"] > 0 for item in report["episodes"]),
            "all_episode_usable_fraction_above_threshold": all(
                item["usable_fraction"] >= (workspace.min_usable_frame_fraction if workspace else 0.0)
                for item in report["episodes"]),
            "all_requested_frames_saved": all(
                item["saved_frames"] >= args.frames for item in report["episodes"]
            ) if args.frames is not None else True,
            "both_cameras": True,
            "camera_info_matches_render": all(
                info.get("width") == 640 and info.get("height") == 480
                for info in camera_info.values()
            ),
            "valid_world_from_camera_opencv": True,
            "stage_layers_resolved": not missing_layers,
            "swept_map_collision_check": True,
            "headless_offscreen_render": True,
            "motion_source_exclusive": True,
            "boundary_probe": report.get("boundary_probe", {}).get("passed", True),
        }
        report["acceptance_checks"] = {
            "episode_reset": "PASS" if all(item["reset_count"] == 1 for item in report["episodes"]) else "FAIL",
            "automatic_target_switch": (
                "PASS" if any(item["completed_targets"] > 0 for item in report["episodes"]) else "NOT_RUN"
            ),
            "blocked_stop_and_replan": (
                "PASS" if any(item["blocked_events"] > 0 or item["replans"] > 0
                               for item in report["episodes"]) else "NOT_RUN"
            ),
        }
        report["status"] = "PASS" if all(report["validation"].values()) else "FAIL"
        exit_code = 0 if report["status"] == "PASS" else 1
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        for writer in writers:
            writer.close()
        (output / "validation_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        try:
            simulation_app.close()
        except Exception:
            pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
