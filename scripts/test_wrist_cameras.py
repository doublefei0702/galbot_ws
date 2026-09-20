#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""阶段 4: 腕部 RGB-D 相机独立采集验证。

流程: 打开调试场景 → 锁臂姿 → 在左右 camera_link 下建 Camera →
物理稳定 → 渲染预热 → 同帧采 RGB + Z深度(distance_to_image_plane) →
读取实际内参 K 与相机世界位姿 → 保存多帧 + camera_info + pose。

用法:
    PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/test_wrist_cameras.py
"""
import json
import importlib.util
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402

W, H = 640, 480
CAM_SPECS = {
    "left_wrist": "/World/robot/left_arm_link7/left_arm_camera_link",
    "right_wrist": "/World/robot/right_arm_link7/right_arm_camera_link",
}
OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "outputs", "wrist_camera_test"))
N_FRAMES = 6
STEERING = [f"wheel_steering_joint{i}" for i in range(1, 5)]
DRIVING = [f"wheel_driving_joint{i}" for i in range(1, 5)]

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_pose_spec = importlib.util.spec_from_file_location(
    "galbot_pose_utils",
    os.path.join(WS, "exts", "galbot.mobility_gen", "galbot", "mobility_gen", "pose_utils.py"))
_pose_utils = importlib.util.module_from_spec(_pose_spec)
_pose_spec.loader.exec_module(_pose_utils)


def main():
    stage_usd = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "stages", "galbot_s1_debug.usd"))
    open_stage(stage_usd)
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0)
    robot = Articulation("/World/robot")
    world.scene.add(robot)
    world.reset()

    names = list(robot.dof_names)
    hold_idx = np.array([i for i, n in enumerate(names) if n not in STEERING + DRIVING])
    idx_steer = np.array([names.index(n) for n in STEERING])
    idx_drive = np.array([names.index(n) for n in DRIVING])

    import yaml
    arm_cfg = yaml.safe_load(open(os.path.join(WS, "configs", "arm_pose.yaml")))
    pose_cmd = arm_cfg.get(arm_cfg.get("default_pose", "nav"), {})
    hold_targets = np.array([float(pose_cmd.get(names[i], 0.0)) for i in hold_idx], dtype=np.float32)
    camera_cfg = yaml.safe_load(open(os.path.join(WS, "configs", "wrist_cameras.yaml")))
    view_cfg = camera_cfg.get("navigation_view", {})

    # 锁定全身(含双臂)目标, 轮子零速 —— 静止采集
    def hold():
        robot.set_joint_position_targets(hold_targets, joint_indices=hold_idx)
        robot.set_joint_position_targets(np.zeros(4, dtype=np.float32), joint_indices=idx_steer)
        robot.set_joint_velocity_targets(np.zeros(4, dtype=np.float32), joint_indices=idx_drive)

    os.makedirs(OUT_DIR, exist_ok=True)
    cameras = {}
    for tag, link in CAM_SPECS.items():
        path = f"{link}/sensors/rgbd_camera"
        cam = Camera(prim_path=path, resolution=(W, H))
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        cameras[tag] = cam
        print(f"已创建 {tag}: {path}")

    for _ in range(400):  # 2s 物理: 下落到轮子+双臂充分收敛
        hold()
        world.step(render=False)
    _, base_q = robot.get_world_poses()
    aim_q = _pose_utils.aimed_world_camera_quat(
        np.asarray(base_q).reshape(-1, 4)[0],
        view_cfg.get("pitch_deg", -10.0), view_cfg.get("yaw_deg", 0.0))
    for cam in cameras.values():
        cam.set_world_pose(orientation=aim_q, camera_axes="world")
    for _ in range(8):  # 渲染预热(annotator 就绪)
        hold()
        world.step(render=True)

    for tag, cam in cameras.items():
        k = np.asarray(cam.get_intrinsics_matrix())
        info = {
            "width": W, "height": H, "K": k.tolist(), "model": "plumb_bob",
            "distortion_model": "rational_polynomial", "D": [0.0] * 5,
            "depth_scale": 1000.0, "depth_units": "distance_to_image_plane(z, meters)",
            "calibration_source": "isaacsim_render",
        }
        with open(os.path.join(OUT_DIR, f"{tag}_camera_info.json"), "w") as f:
            json.dump(info, f, indent=2)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        print(f"{tag} 实际K: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} (期望 cx≈{(W-1)/2:.0f} cy≈{(H-1)/2:.0f})")

    for i in range(N_FRAMES):
        world.step(render=True)
        for tag, cam in cameras.items():
            rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
            depth = np.asarray(cam.get_current_frame().get("distance_to_image_plane"), dtype=np.float32)
            # camera_axes='ros' 已经是 OpenCV optical (+X右,+Y下,+Z前)，勿二次转换。
            pos, quat = cam.get_world_pose(camera_axes="ros")
            T_cv = _pose_utils.pose_matrix(pos, quat)

            Image.fromarray(rgb).save(os.path.join(OUT_DIR, f"{tag}_rgb_{i:03d}.png"))
            np.save(os.path.join(OUT_DIR, f"{tag}_depth_{i:03d}.npy"), depth)
            d16 = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
            Image.fromarray(d16, mode="I;16").save(os.path.join(OUT_DIR, f"{tag}_depth16_{i:03d}.png"))
            with open(os.path.join(OUT_DIR, f"{tag}_pose_{i:03d}.txt"), "w") as f:
                f.write("# world_from_camera_opencv 4x4\n" + "\n".join(" ".join(f"{v:.6f}" for v in row) for row in T_cv) + "\n")
            if i == 0:
                _, world_quat = cam.get_world_pose(camera_axes="world")
                forward, pitch = _pose_utils.camera_forward_pitch(world_quat)
                valid = depth[np.isfinite(depth) & (depth < 100)]
                if valid.size == 0:
                    valid = np.array([float('nan')])
                print(f"{tag} 帧{i}: RGB均值={rgb.mean():.1f} | 深度 min={valid.min():.2f} p50={np.percentile(valid,50):.2f} "
                      f"max={valid.max():.2f} m | 相机z={pos[2]:.3f} 俯角={pitch:+.1f}° 前向={forward.round(3)}")

    print(f"\n已保存 {N_FRAMES} 帧到 {OUT_DIR}")
    simulation_app.close()


if __name__ == "__main__":
    main()
