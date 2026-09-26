#!/usr/bin/env python3
"""Render fixed camera poses and save RGB, raw Z depth, mask and point cloud.

Run once with local_source_only.usd and once with local_work_scene.usd.
The configured poses are diagnostic views, not claims that a wall/floor defect exists.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scene", type=Path, required=True)
parser.add_argument("--workspace-config", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp(launch_config={"headless": True})

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import UsdGeom  # noqa: E402
import omni.usd  # noqa: E402
import sys  # noqa: E402

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "exts/galbot.mobility_gen/galbot/mobility_gen"))
from local_workspace import load_workspace  # noqa: E402
from pose_utils import matrix_to_quat_wxyz, pose_matrix, set_camera_horizontal_fov  # noqa: E402
import yaml  # noqa: E402

def aim_quat(position, target):
    dx, dy, dz = np.subtract(target, position)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(-pitch), math.sin(-pitch)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return matrix_to_quat_wxyz(rz @ ry)


def main():
    workspace = load_workspace(args.workspace_config)
    cfg = yaml.safe_load(Path(args.workspace_config).read_text(encoding="utf-8"))
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    open_stage(str(args.scene.expanduser().resolve()))
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=1 / 60)
    world.reset()
    camera = Camera(prim_path="/World/ValidationCamera", resolution=(640, 480))
    camera.initialize()
    set_camera_horizontal_fov(camera, float(cfg["camera_horizontal_fov_deg"]))
    camera.set_clipping_range(workspace.depth_min_m, workspace.depth_max_m)
    camera.add_distance_to_image_plane_to_frame()
    rows = []
    for name, spec in cfg["validation_views"].items():
        position = spec["position_world"]
        target = spec["target_world"]
        camera.set_world_pose(position=np.array(position), orientation=aim_quat(position, target), camera_axes="world")
        for _ in range(12):
            world.step(render=True)
        rgba = np.asarray(camera.get_rgba())
        depth = np.asarray(camera.get_current_frame()["distance_to_image_plane"], dtype=np.float32).squeeze()
        if depth.shape != (480, 640):
            raise RuntimeError(f"{name}: unexpected depth shape {depth.shape}")
        rgb = rgba[:, :, :3].astype(np.uint8)
        valid = np.isfinite(depth) & (depth >= workspace.depth_min_m) & (depth <= workspace.depth_max_m)
        k = np.asarray(camera.get_intrinsics_matrix(), dtype=float)
        pos, quat = camera.get_world_pose(camera_axes="ros")
        matrix = pose_matrix(np.asarray(pos), np.asarray(quat))
        yy, xx = np.mgrid[0:480:4, 0:640:4]
        z = depth[::4, ::4]
        use = valid[::4, ::4]
        z_safe = np.where(use, z, 0.0)
        optical = np.stack(((xx-k[0, 2])*z_safe/k[0, 0],
                            (yy-k[1, 2])*z_safe/k[1, 1], z_safe), axis=-1)[use]
        points = optical @ matrix[:3, :3].T + matrix[:3, 3]
        folder = output / name
        folder.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb, mode="RGB").save(folder / "rgb.png")
        np.save(folder / "depth_raw.npy", depth)
        Image.fromarray(valid.astype(np.uint8)*255, mode="L").save(folder / "depth_valid_mask.png")
        np.save(folder / "points_world.npy", points.astype(np.float32))
        display = np.zeros(depth.shape, dtype=np.uint8)
        display[valid] = np.clip(255.0 * (1.0 - depth[valid] / workspace.depth_max_m), 0, 255).astype(np.uint8)
        Image.fromarray(display, mode="L").save(folder / "depth_preview.png")
        row = {"view": name, "position_world": position, "target_world": target,
               "camera_world_from_opencv": matrix.tolist(), "K": k.tolist(),
               "clipping_range_m": list(camera.get_clipping_range()),
               "valid_fraction": float(valid.mean()), "points": int(len(points)),
               "depth_p50_m": float(np.median(depth[valid])) if valid.any() else None,
               "depth_min_m": float(depth[valid].min()) if valid.any() else None,
               "depth_max_m": float(depth[valid].max()) if valid.any() else None}
        rows.append(row)
        print(json.dumps(row))
    (output / "summary.json").write_text(json.dumps({"scene": str(args.scene), "views": rows}, indent=2) + "\n")
    simulation_app.close()


if __name__ == "__main__":
    main()
