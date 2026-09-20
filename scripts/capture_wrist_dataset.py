#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""阶段 5: 仓库环境行驶中的腕部 RGB-D 同步采集。

场景: 官方 warehouse_with_forklifts + Galbot S1(仿真变体, 浮动基座)。
运动: 直线前进(可用 --motion 选择 straight|square|spin)。
每帧严格顺序: 物理步 → render → 同帧读 RGB/深度/相机位姿/K → 机器人状态 → 落盘。

输出(用户规范布局):
outputs/scene_NNN/{left_wrist,right_wrist}/
    rgb/*.png  depth/*.png(uint16)  depth_raw/*.npy(float32米)
    camera_info.json  manifest.jsonl  pose.txt(每帧4x4)

用法:
    PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/capture_wrist_dataset.py \
        [--motion straight] [--frames 48] [--speed 0.25]
"""
import argparse
import importlib.util
import json
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.prims import add_reference_to_stage  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "swerve_controller",
    os.path.join(os.path.dirname(__file__), "..", "exts", "galbot.mobility_gen",
                 "galbot", "mobility_gen", "swerve_controller.py"))
_mod = importlib.util.module_from_spec(_spec)
sys.modules["swerve_controller"] = _mod
_spec.loader.exec_module(_mod)
SwerveController = _mod.SwerveController

_pose_spec = importlib.util.spec_from_file_location(
    "galbot_pose_utils",
    os.path.join(os.path.dirname(__file__), "..", "exts", "galbot.mobility_gen",
                 "galbot", "mobility_gen", "pose_utils.py"))
_pose_utils = importlib.util.module_from_spec(_pose_spec)
_pose_spec.loader.exec_module(_pose_utils)

WAREHOUSE = "/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/warehouse_with_forklifts_edit.usd"
GALBOT_SIM = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "stages", "galbot_s1_sim_src", "galbot_s1.usda"))

W, H = 640, 480
CAM_SPECS = {
    "left_wrist": "/World/robot/left_arm_link7/left_arm_camera_link",
    "right_wrist": "/World/robot/right_arm_link7/right_arm_camera_link",
}
STEERING = [f"wheel_steering_joint{i}" for i in range(1, 5)]
DRIVING = [f"wheel_driving_joint{i}" for i in range(1, 5)]
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion", choices=["straight", "square", "spin"], default="straight")
    parser.add_argument("--frames", type=int, default=48)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--scene-name", default=None)
    args, _ = parser.parse_known_args()

    # 输出目录
    base = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "outputs"))
    n = 1
    while os.path.exists(os.path.join(base, f"scene_{n:03d}")):
        n += 1
    scene_dir = os.path.join(base, args.scene_name or f"scene_{n:03d}")
    for tag in CAM_SPECS:
        for sub in ("rgb", "depth", "depth_raw"):
            os.makedirs(os.path.join(scene_dir, tag, sub), exist_ok=True)

    # 预生成组合场景(离线脚本已写好变体与出生位姿), open_stage 模式驱动写入已验证
    CAPTURE_STAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "stages", "galbot_warehouse_capture.usd"))
    from isaacsim.core.utils.stage import open_stage  # noqa: E402

    open_stage(CAPTURE_STAGE)
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0)
    robot = Articulation("/World/robot")
    world.scene.add(robot)
    world.reset()

    import yaml
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "configs", "arm_pose.yaml")))
    pose_cmd = {k: float(v) for k, v in cfg.get(cfg.get("default_pose", "nav"), {}).items()}
    camera_cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "configs", "wrist_cameras.yaml")))
    view_cfg = camera_cfg.get("navigation_view", {})

    names = list(robot.dof_names)
    idx_steer = np.array([names.index(n) for n in STEERING])
    idx_drive = np.array([names.index(n) for n in DRIVING])
    hold_idx = np.array([i for i, n in enumerate(names) if n not in STEERING + DRIVING])
    hold_targets = np.array([pose_cmd.get(names[i], 0.0) for i in hold_idx], dtype=np.float32)
    controller = SwerveController(wheel_radius=0.08)

    cameras = {}
    for tag, link in CAM_SPECS.items():
        path = f"{link}/sensors/rgbd_camera"
        cam = Camera(prim_path=path, resolution=(W, H))
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        cameras[tag] = cam

    def hold_all():
        robot.set_joint_position_targets(hold_targets, joint_indices=hold_idx)

    for _ in range(400):  # 2s 稳定落地，并让双臂充分收敛后再标定本地旋转
        hold_all()
        world.step(render=False)
    _, base_q = robot.get_world_poses()
    aim_q = _pose_utils.aimed_world_camera_quat(
        np.asarray(base_q).reshape(-1, 4)[0],
        view_cfg.get("pitch_deg", -10.0), view_cfg.get("yaw_deg", 0.0))
    for cam in cameras.values():
        cam.set_world_pose(orientation=aim_q, camera_axes="world")
    for _ in range(6):
        hold_all()
        world.step(render=True)

    # 写 camera_info (实际渲染 K)
    K = None
    for tag, cam in cameras.items():
        k = np.asarray(cam.get_intrinsics_matrix())
        K = k
        info = {"width": W, "height": H, "K": k.tolist(), "model": "plumb_bob",
                "D": [0.0] * 5, "depth_scale": 1000.0,
                "depth_units": "distance_to_image_plane(z, meters)",
                "calibration_source": "isaacsim_render"}
        with open(os.path.join(scene_dir, tag, "camera_info.json"), "w") as f:
            json.dump(info, f, indent=2)

    manifests = {tag: open(os.path.join(scene_dir, tag, "manifest.jsonl"), "w") for tag in CAM_SPECS}
    pose_files = {tag: open(os.path.join(scene_dir, tag, "pose.txt"), "w") for tag in CAM_SPECS}
    step_per_frame = 12  # 60fps渲染 × 每12步采1帧 = 5Hz

    frame = 0
    total_steps = args.frames * step_per_frame
    for step in range(total_steps):
        t = step / total_steps
        if args.motion == "straight":
            cmd = [args.speed, 0.0, 0.0]
        elif args.motion == "spin":
            cmd = [0.0, 0.0, 0.4]
        else:  # square: 每段转向90°
            seg = int(t * 4) % 4
            cmd = [args.speed, 0.0, 0.8 if (step % (total_steps // 4)) > (total_steps // 4 - 100) else 0.0]
        hold_all()
        pos = np.asarray(robot.get_joint_positions()).flatten()
        targets = controller.forward_with_state(np.array(cmd), list(pos[idx_steer]))
        steer_t = np.array([t_[0] for t_ in targets], dtype=np.float32)
        drive_w = np.array([t_[1] for t_ in targets], dtype=np.float32)
        robot.set_joint_position_targets(steer_t, joint_indices=idx_steer)
        robot.set_joint_velocity_targets(drive_w, joint_indices=idx_drive)
        world.step(render=(step % step_per_frame == step_per_frame - 1) or step < 6)
        if step % step_per_frame == step_per_frame - 1:
            base_p, base_q = robot.get_world_poses()
            base_p = np.asarray(base_p).flatten()[:3]
            sim_t = (step + 1) * world.get_physics_dt()
            for tag, cam in cameras.items():
                rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
                depth = np.asarray(cam.get_current_frame().get("distance_to_image_plane"), dtype=np.float32)
                # Explicit OpenCV/ROS optical axes: +X right, +Y down, +Z forward.
                cpos, cquat = cam.get_world_pose(camera_axes="ros")
                T_cv = _pose_utils.pose_matrix(cpos, cquat)
                Image.fromarray(rgb).save(os.path.join(scene_dir, tag, "rgb", f"{frame:05d}.png"))
                np.save(os.path.join(scene_dir, tag, "depth_raw", f"{frame:05d}.npy"), depth)
                d16 = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
                Image.fromarray(d16, mode="I;16").save(os.path.join(scene_dir, tag, "depth", f"{frame:05d}.png"))
                valid = depth[np.isfinite(depth) & (depth < 100)]
                meta = {
                    "frame": frame, "sim_time": float(sim_t), "base_pose_xyz": base_p.tolist(),
                    "wheel_steer": pos[idx_steer].tolist(), "wheel_drive": pos[idx_drive].tolist(),
                    "camera_pos": np.asarray(cpos).tolist(),
                    "depth_min": float(valid.min()) if valid.size else None,
                    "depth_p50": float(np.percentile(valid, 50)) if valid.size else None,
                    "rgb_mean": float(rgb.mean()),
                }
                manifests[tag].write(json.dumps(meta) + "\n")
                pose_files[tag].write(f"# frame {frame}\n" + "\n".join(" ".join(f"{v:.6f}" for v in r) for r in T_cv) + "\n")
            frame += 1
            if frame % 12 == 0:
                print(f"进度 {frame}/{args.frames} 基座=[{base_p[0]:+.2f},{base_p[1]:+.2f},{base_p[2]:.3f}]")

    for f in manifests.values():
        f.close()
    for f in pose_files.values():
        f.close()
    print(f"完成: {frame} 帧已保存到 {scene_dir}")
    simulation_app.close()


if __name__ == "__main__":
    main()
