#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""阶段 6: Galbot S1 × MobilityGen 无头集成验证。

链路: 注册 GalbotS1Robot → 仓库场景 → 构建 → RandomPathFollowing 随机路径
跟随 → 全程录制 → 校验录制数据可被 replay 读取。

用法:
    PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/test_mobility_gen_galbot.py \
        --enable isaacsim.replicator.mobility_gen.examples --steps 2000
"""
import asyncio
import os
import shutil
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import numpy as np  # noqa: E402

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(WS, "exts", "galbot.mobility_gen"))  # galbot 包的父目录

WAREHOUSE = "/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/warehouse_with_forklifts_edit.usd"
OMAP = "/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/MobilityGenData/map.yaml"
N_STEPS = int(os.environ.get("MG_STEPS", "2000"))


def main():
    # 1) 注册检查
    import galbot.mobility_gen  # noqa: F401  触发 ROBOTS.register()
    from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS

    names = list(ROBOTS.names())
    assert "GalbotS1Robot" in names, f"GalbotS1Robot 未注册! 已注册: {names}"
    print(f"1. GalbotS1Robot 已注册 (共 {len(names)} 个机器人: {names})")

    # 2) 打开仓库 + 构建 world/机器人/场景 (复刻 UI build 流程)
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.replicator.mobility_gen.impl.occupancy_map import OccupancyMap
    from isaacsim.replicator.mobility_gen.impl.config import Config
    from isaacsim.replicator.mobility_gen.impl.reader import MobilityGenReader
    from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world, new_world
    from isaacsim.replicator.mobility_gen.impl.writer import MobilityGenWriter
    from galbot.mobility_gen.robots import GalbotS1Robot

    print("   [dbg] open_stage...", flush=True)
    open_stage(WAREHOUSE)
    print("   [dbg] new_world...", flush=True)
    world = new_world(physics_dt=GalbotS1Robot.physics_dt)
    print("   [dbg] build robot...", flush=True)
    robot = GalbotS1Robot.build("/World/robot")
    print("   [dbg] world.reset...", flush=True)
    world.reset()  # 无头脚本必须用同步 reset; run_until_complete(async版) 会与 kit 事件循环死锁
    print(f"2. 机器人已构建, DOF={len(list(robot.articulation_view.dof_names))} (模拟UI流程: 不手动initialize)")

    omap = OccupancyMap.from_ros_yaml(OMAP)
    from isaacsim.replicator.mobility_gen.examples.scenarios import RandomPathFollowingScenario

    scenario = RandomPathFollowingScenario.from_robot_occupancy_map(robot, omap)
    scenario.reset()
    print(f"3. 场景已构建, 初始位姿 {robot.get_pose_2d()}")

    # 4) 先让双臂收敛并完成腕部相机定向。预热帧不进入正式录制。
    dt = world.get_physics_dt()
    for _ in range(400):
        robot.write_action(dt)
        world.step(render=False)
    scenario.reset()

    # 5) 初始化一个完整的 MobilityGen recording。replay_directory.py 除状态帧外
    # 还要求 config.json、stage.usd 和 occupancy_map/map.yaml。
    rec_dir = os.path.join(os.path.expanduser("~/MobilityGenData"), "recordings", "galbot_s1_headless_test")
    if os.path.isdir(rec_dir):
        shutil.rmtree(rec_dir)
    writer = MobilityGenWriter(rec_dir)
    writer.write_config(Config(
        scenario_type="RandomPathFollowingScenario",
        robot_type="GalbotS1Robot",
        scene_usd=WAREHOUSE,
    ))
    writer.write_occupancy_map(omap)
    writer.copy_stage(WAREHOUSE)

    # 6) 运行 + 录制
    poses = []
    resets = 0
    for i in range(N_STEPS):
        alive = scenario.step(dt)
        world.step(render=False)
        if i % 10 == 0:
            writer.write_state_dict_common(scenario.state_dict_common(), step=i // 10)
            poses.append(robot.get_pose_2d())
        if not alive:
            scenario.reset()
            resets += 1
        if i % 400 == 0:
            p = robot.get_pose_2d()
            print(f"   step {i}: 位姿 x={p.x:+.2f} y={p.y:+.2f} θ={np.degrees(p.theta):+.0f}°")

    xs = [p.x for p in poses]
    ys = [p.y for p in poses]
    travel = float(np.sum(np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)))
    print(f"4. 完成 {N_STEPS} 步: 累计里程 {travel:.2f}m, 场景重置 {resets} 次, 轨迹点 {len(poses)}")
    print(f"   录制目录: {rec_dir}")
    assert travel > 1.0, "机器人没有实际移动!"
    reader = MobilityGenReader(rec_dir)
    assert len(reader) == len(poses), "MobilityGen recording 状态帧数量不一致"
    reader.read_config()
    reader.read_occupancy_map()
    for required in ("stage.usd", "config.json", os.path.join("occupancy_map", "map.yaml")):
        assert os.path.isfile(os.path.join(rec_dir, required)), f"录制初始化文件缺失: {required}"
    print("\n集成验证通过 ✔")
    simulation_app.close()


if __name__ == "__main__":
    main()
