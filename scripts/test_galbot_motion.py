#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""阶段 3: Galbot S1 底盘独立移动测试（Swerve 控制验收）。

测试序列: 静止5s → 直行 → 后退 → 原地旋转 → 定半径转弯 → 停止 → 高度检查。
双臂保持建图姿态(位置目标锁定)，只控制 4 转向 + 4 驱动关节。
关节按名称解析索引，不依赖 USD 排列顺序。

用法:
    /root/isaacsim/python.sh scripts/test_galbot_motion.py [--stage <usd>]
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402


import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "swerve_controller",
    os.path.join(os.path.dirname(__file__), "..", "exts", "galbot.mobility_gen",
                 "galbot", "mobility_gen", "swerve_controller.py"),
)
_mod = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _mod
_spec.loader.exec_module(_mod)
SwerveController = _mod.SwerveController

STEERING_JOINTS = ["wheel_steering_joint1", "wheel_steering_joint2",
                   "wheel_steering_joint3", "wheel_steering_joint4"]
DRIVING_JOINTS = ["wheel_driving_joint1", "wheel_driving_joint2",
                  "wheel_driving_joint3", "wheel_driving_joint4"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=os.path.join(os.path.dirname(__file__), "..", "stages", "galbot_s1_debug.usd"))
    parser.add_argument("--wheel-radius", type=float, default=0.08)
    args, _ = parser.parse_known_args()

    # 直接打开调试场景（场景内已有 /World/robot 与带碰撞的地面），
    # 不要再 add_reference_to_stage —— 那样会得到 /World/robot/robot 双重嵌套。
    # 必须先 open_stage 再创建 World，否则 World 内部 scene 与新 stage 脱钩
    from isaacsim.core.utils.stage import open_stage
    open_stage(os.path.abspath(args.stage))
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0)

    robot = Articulation("/World/robot")
    world.scene.add(robot)
    world.reset()

    names = list(robot.dof_names)
    print(f"DOF 数量: {len(names)}")
    print("关节名:", names)
    idx_steer = np.array([names.index(n) for n in STEERING_JOINTS])
    idx_drive = np.array([names.index(n) for n in DRIVING_JOINTS])
    hold_idx = [i for i, n in enumerate(names)
                if n not in STEERING_JOINTS and n not in DRIVING_JOINTS]
    hold_targets = np.zeros(len(hold_idx), dtype=np.float32)  # 建图姿态全零(后续替换)
    # 先写一次保持目标
    robot.set_joint_position_targets(hold_targets, joint_indices=np.array(hold_idx))

    controller = SwerveController(wheel_radius=args.wheel_radius)

    def run(cmd, steps):
        for _ in range(steps):
            pos = np.asarray(robot.get_joint_positions()).flatten()
            targets = controller.forward_with_state(np.array(cmd), list(pos[idx_steer]))
            steer_t = np.array([t[0] for t in targets], dtype=np.float32)
            drive_w = np.array([t[1] for t in targets], dtype=np.float32)
            robot.set_joint_position_targets(steer_t, joint_indices=idx_steer)
            robot.set_joint_velocity_targets(drive_w, joint_indices=idx_drive)
            world.step(render=False)

    def pose():
        p, _ = robot.get_world_poses()
        return np.asarray(p[0] if np.ndim(p) == 2 else p).flatten()[:3]

    def report(tag, before, after, note=""):
        d = after - before
        print(f"{tag}: d=[{d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}] {note}")

    b = pose(); run([0, 0, 0], 1000); report("静止5s      ", b, pose(), "(期望≈0)")
    b = pose(); run([0.3, 0, 0], 600); report("直行3s      ", b, pose(), "(期望dx≈+0.9)")
    b = pose(); run([-0.3, 0, 0], 600); report("后退3s      ", b, pose(), "(期望dx≈-0.9)")
    b = pose(); run([0, 0, 0.5], 600); report("原地旋转3s  ", b, pose(), "(期望xy≈0)")
    b = pose(); run([0.3, 0, 0.3], 600); report("定半径转弯3s", b, pose(), "")
    b = pose(); run([0, 0, 0], 400); report("停止2s      ", b, pose(), "(期望≈0)")

    p = pose()
    print(f"最终基座位置: [{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}]  z 用于塌陷检查")
    jp = np.asarray(robot.get_joint_positions()).flatten()
    print("轮驱动关节位置(rad):", jp[idx_drive].round(2))
    print("轮转向关节位置(rad):", jp[idx_steer].round(3))

    simulation_app.close()


if __name__ == "__main__":
    main()
