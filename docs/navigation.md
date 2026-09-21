# 与导航栈对接

面向把 GalbotS1 接入自己导航系统（ROS 2 Nav2、自研规划器等）的用户。
本仓库**不包含 ROS 桥**：仿真侧的对接面是 Python API——导航系统把速度指令
（cmd_vel 语义）写入 `robot.action`，从仿真读取位姿与传感器数据。

```text
导航系统（规划/控制）                    仿真侧（本仓库）
┌──────────────────┐   速度指令 [v, wz]  ┌─────────────────────────┐
│  外部规划器/Nav2  │ ─────────────────→ │ robot.action + write_action │
│                  │   位姿 Pose2d      │ GalbotS1Robot (swerve)   │
│                  │ ←───────────────── │ robot.get_pose_2d()      │
│                  │   腕部 RGB-D       │ robot.{left,right}_wrist │
│                  │   占据图           │ OccupancyMap (map.yaml)  │
└──────────────────┘                    └─────────────────────────┘
```

## 控制接口：接收 cmd_vel

`robot.action` 是一个 `Buffer`，值为 `[v, wz]`（m/s、rad/s），语义等价于
`geometry_msgs/Twist` 的 `linear.x` 与 `angular.z`（差速模型；swerve 底盘
物理上支持横向速度，但该接口未使用 `vy`）。

两个要点：

1. **`set_value` 只是写入缓冲**，机器人不会动。每个物理步必须调用
   `robot.write_action(dt)` 才会把指令转换成四轮转向/驱动目标下发，
   同时保持臂姿和触发相机定向。
2. 指令刷新频率不必等于物理频率（200 Hz），但 `write_action` 每个物理步
   都要调用。低频刷新时指令在两次刷新之间保持不变。

无头脚本中的最小驱动骨架（完整可运行参考
`scripts/test_mobility_gen_galbot.py`）：

```python
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import new_world
from galbot.mobility_gen.robots import GalbotS1Robot

world = new_world(physics_dt=GalbotS1Robot.physics_dt)   # 200 Hz
robot = GalbotS1Robot.build("/World/robot")              # 打开你的场景后
world.reset()

robot.action.set_value(np.array([0.3, 0.0]))   # cmd_vel: v=0.3 m/s, wz=0
while running:
    robot.write_action(world.get_physics_dt())
    world.step(render=need_render)
```

**ROS 桥的建议形态**：订阅 `/cmd_vel`，回调中只缓存 Twist；在物理步循环
（或 Isaac Sim 的 physics callback）里把缓存的 `linear.x`/`angular.z` 写入
`robot.action` 并调用 `write_action`。这样指令频率与物理节拍解耦，避免
回调时序问题。Isaac Sim 自带 ROS 2 桥扩展可用于话题搬运，但尚未集成进
本仓库——上述 action 接口就是预留的接入点。

GUI 模式下 `KeyboardTeleoperation` 场景即用同一接口：按键映射成
`[±keyboard_linear_velocity_gain, ±keyboard_angular_velocity_gain]` 后
`set_value`。遥控增益、路径跟随参数都是 `GalbotS1Robot` 类属性，可子类化
覆盖。

## 状态反馈：里程计

- `robot.get_pose_2d()` → `Pose2d(x, y, theta)`：基座**物理真值位姿**，
  非轮式里程计。坐标约定 +x 前、+y 左、θ 绕 +z 逆时针为正（REP-103）。
- `robot.update_state()` 刷新内部 Buffer：`position`、`orientation`
  （scalar-first 四元数）、`joint_positions`、`joint_velocities`。
- **注意**：当前实现中 `linear_velocity`/`angular_velocity` 两个 Buffer 固定
  写零。需要速度反馈时对 `get_pose_2d()` 差分，或直接从
  `robot.articulation_view` 读取刚体速度。
- 腕部相机逐帧世界位姿（OpenCV optical 约定）与 RGB-D 可作为视觉导航
  输入；相机视野在 Build 后约 2 s（双臂稳定）才完成定向。

## 目标点导航：给定 target pose 走过去

MobilityGen 自带场景只有键盘/手柄遥控、随机加速度和**随机**路径跟随，
没有"给定目标位姿"的现成 scenario。两条实现路径：

### 路径 A：外部规划器（推荐对接 Nav2 等成熟栈）

1. **代价地图底图**：`OccupancyMap.from_ros_yaml(map.yaml)`，其
   `freespace_mask()`、`buffered_meters(radius)` 提供占据/自由空间；
   生成占据参数见 `GalbotS1Robot` 类属性（采样半径 0.5 m、高度带
   0.05–0.5 m、格子 0.05 m、碰撞半径 0.45 m）。
2. **定位**：`get_pose_2d()` 真值，直接当完美里程计用。
3. **规划与控制**全部在外部完成，速度指令按上一节写入 `action`。

### 路径 B：仿真内路径跟随（复用 MobilityGen 工具）

`isaacsim.replicator.mobility_gen.impl` 提供了路径规划与跟随的全部构件
（`RandomPathFollowingScenario` 即用它们实现随机目标）：

```python
from isaacsim.replicator.mobility_gen.impl.path_planner import (
    generate_paths, compress_path)
from isaacsim.replicator.mobility_gen.impl.utils.path_utils import PathHelper

# 1. 规划：当前位姿 → 目标点（占据图像素坐标）
px = omap.world_to_pixel_numpy([[pose.x, pose.y]])
out = generate_paths((px[0, 1], px[0, 0]),
                     omap.buffered_meters(robot.occupancy_map_radius).freespace_mask())
end_px = 目标点像素坐标                              # 自己指定 target pose
path = out.unroll_path(end_px)
path, _ = compress_path(path)
path = omap.pixel_to_world_numpy(path[:, ::-1])      # 转世界坐标 (x, y)

# 2. 跟随：前视点 + 角度差 → [v, wz]
helper = PathHelper(path)
pt, s, _, _ = helper.find_nearest([pose.x, pose.y])
target = helper.get_point_by_distance(s + robot.path_following_target_point_offset_meters)
d_theta = 目标方向与当前朝向的角度差
robot.action.set_value([robot.path_following_speed,
                        robot.path_following_angular_gain * d_theta])
```

控制律细节（前视距离、到位阈值、朝向阈值判断）参考
`RandomPathFollowingScenario.step()` 源码；跟随参数（速度 0.3 m/s、角增益
1.5、到位阈值 0.5 m 等）是 `GalbotS1Robot` 类属性，可按需覆盖。

## 对接注意事项

- **坐标约定**：REP-103（+x 前 +y 左，θ 逆时针）；四元数 wxyz；相机外参
  OpenCV optical。与 ROS 交换数据时无需额外变换，但不要混用 USD 相机轴。
- **时间**：物理 200 Hz、渲染 60 Hz，`sim_time` 从物理步累计；外部控制器
  用 `world.get_physics_dt()` 对齐。
- **速度执行误差**：仓库地面存在打滑，`wheel_radius=0.08 m` 是估计值，
  实际速度会低于指令值。闭环（用 `get_pose_2d()` 反馈）比开环可靠。
- **传感器面**：只有双腕 RGB-D，没有激光雷达、IMU、保险杠传感器；
  视觉方案需考虑腕部视野随臂姿固定的前下视角（pitch -10°）。
- **场景更换**：换环境只改场景 USD + `map.yaml`（见
  [configuration.md](configuration.md#场景与路径自定义)），机器人接口不变。
- **重置**：`robot.set_pose_2d(pose)` 直接传送机器人并清零速度，适合
  导航测试的初始摆位或失败恢复。
