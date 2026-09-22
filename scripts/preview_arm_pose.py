#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Preview both wrist cameras with an explicit base-relative navigation view.

The arms choose camera positions only. After they settle, both cameras are aimed
relative to the robot base with Isaac Sim's documented ``world`` camera axes
(+X forward, +Z up). This avoids orientation IK through mirrored arm chains.

Examples:
  ISP scripts/preview_arm_pose.py
  ISP scripts/preview_arm_pose.py --pitch -25 --yaw 0
  ISP scripts/preview_arm_pose.py --pitch -25 --save-view
"""
import argparse
import importlib.util
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
W, H = 640, 480
CAM_SPECS = {
    "left": "left_arm_link7/left_arm_camera_link",
    "right": "right_arm_link7/right_arm_camera_link",
}
STEERING = [f"wheel_steering_joint{i}" for i in range(1, 5)]
DRIVING = [f"wheel_driving_joint{i}" for i in range(1, 5)]

_pose_spec = importlib.util.spec_from_file_location(
    "galbot_pose_utils",
    os.path.join(WS, "exts", "galbot.mobility_gen", "galbot", "mobility_gen", "pose_utils.py"))
pose_utils = importlib.util.module_from_spec(_pose_spec)
_pose_spec.loader.exec_module(pose_utils)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--poses", default=None, help="arm_pose.yaml 姿态名，逗号分隔；默认 default_pose")
    parser.add_argument("--pitch", type=float, default=None, help="视线俯角，负值向下；默认读取配置")
    parser.add_argument("--yaw", type=float, default=None, help="视线偏航，正值向左；默认读取配置")
    parser.add_argument("--fov", type=float, default=None, help="水平视场角（度）；越大看得越广")
    parser.add_argument("--save-view", action="store_true", help="把 pitch/yaw/fov 保存到 wrist_cameras.yaml")
    parser.add_argument("--stage", default=os.path.join(WS, "stages", "galbot_warehouse_capture.usd"))
    parser.add_argument("--out", default=os.path.join(WS, "outputs", "arm_pose_preview"))
    args, _ = parser.parse_known_args()

    arm_path = os.path.join(WS, "configs", "arm_pose.yaml")
    camera_path = os.path.join(WS, "configs", "wrist_cameras.yaml")
    with open(arm_path) as stream:
        arm_cfg = yaml.safe_load(stream)
    with open(camera_path) as stream:
        camera_cfg = yaml.safe_load(stream)
    view = camera_cfg.setdefault("navigation_view", {})
    pitch = float(args.pitch if args.pitch is not None else view.get("pitch_deg", -10.0))
    yaw = float(args.yaw if args.yaw is not None else view.get("yaw_deg", 0.0))
    fov = float(args.fov if args.fov is not None else view.get("horizontal_fov_deg", 70.0))
    if args.save_view:
        view.update({"pitch_deg": pitch, "yaw_deg": yaw, "horizontal_fov_deg": fov})
        with open(camera_path, "w") as stream:
            yaml.safe_dump(camera_cfg, stream, allow_unicode=True, sort_keys=False)
        print(f"已保存导航视线: pitch={pitch:+.1f}°, yaw={yaw:+.1f}°, HFOV={fov:.1f}°")

    pose_names = (args.poses or arm_cfg.get("default_pose", "nav")).split(",")
    poses = {name: arm_cfg[name] for name in pose_names}
    os.makedirs(args.out, exist_ok=True)

    open_stage(args.stage)
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0)
    robot = Articulation("/World/robot")
    world.scene.add(robot)
    world.reset()

    names = list(robot.dof_names)
    idx_steer = np.array([names.index(name) for name in STEERING])
    idx_drive = np.array([names.index(name) for name in DRIVING])
    hold_idx = np.array([i for i, name in enumerate(names) if name not in STEERING + DRIVING])

    cameras = {}
    for side, rel_path in CAM_SPECS.items():
        camera = Camera(
            prim_path=f"/World/robot/{rel_path}/sensors/rgbd_camera", resolution=(W, H))
        camera.initialize()
        pose_utils.set_camera_horizontal_fov(camera, fov)
        camera.add_distance_to_image_plane_to_frame()
        cameras[side] = camera

    for pose_name, pose in poses.items():
        targets = np.array([float(pose.get(names[i], 0.0)) for i in hold_idx], dtype=np.float32)
        for _ in range(400):
            robot.set_joint_position_targets(targets, joint_indices=hold_idx)
            robot.set_joint_velocity_targets(np.zeros(4, dtype=np.float32), joint_indices=idx_drive)
            robot.set_joint_position_targets(np.zeros(4, dtype=np.float32), joint_indices=idx_steer)
            world.step(render=False)

        _, base_q = robot.get_world_poses()
        aim_q = pose_utils.aimed_world_camera_quat(
            np.asarray(base_q).reshape(-1, 4)[0], pitch, yaw)
        for camera in cameras.values():
            camera.set_world_pose(orientation=aim_q, camera_axes="world")
        for _ in range(8):
            world.step(render=True)

        for side, camera in cameras.items():
            rgb = np.asarray(camera.get_rgba())[:, :, :3].astype(np.uint8)
            depth = np.asarray(camera.get_current_frame().get("distance_to_image_plane"), dtype=np.float32)
            position, quat = camera.get_world_pose(camera_axes="world")
            forward, measured_pitch = pose_utils.camera_forward_pitch(quat)
            valid = depth[np.isfinite(depth) & (depth < 100)]
            p50 = float(np.percentile(valid, 50)) if valid.size else float("nan")
            output = os.path.join(args.out, f"{pose_name}_{side}_wrist.png")
            Image.fromarray(rgb).save(output)
            print(f"[{pose_name}/{side}] z={position[2]:.2f}m pitch={measured_pitch:+.1f}° HFOV={fov:.1f}° "
                  f"forward={forward.round(3)} depth_p50={p50:.2f}m rgb_mean={rgb.mean():.0f}")

    print(f"预览图已保存到 {args.out}")
    simulation_app.close()


if __name__ == "__main__":
    main()
