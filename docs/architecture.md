# 实现思路

本文说明各模块的设计动机与关键实现。操作步骤见 [workflows.md](workflows.md)。

## 总体结构

工程分三层：

1. **资产生成层**（`scripts/`，系统 Python + usd-core）：从官方只读资产派生出
   可移动的仿真 USD，并组装调试 / 采集场景。
2. **运行时层**（Isaac Sim Python）：独立验收脚本（底盘、相机）与同步采集脚本。
3. **扩展层**（`exts/galbot.mobility_gen/`）：把机器人注册进 MobilityGen 框架，
   复用其遥控、路径跟随、录制和 replay 能力。

`swerve_controller.py` 和 `pose_utils.py` 不依赖 Isaac Sim，可被前两层通过
`importlib` 直接加载，也可用 `tests/` 离线验证。

## 仿真资产派生（make_sim_variant.py）

官方 `galbot_s1_description` 的 USD 无法直接用于移动仿真，存在三个问题，
派生脚本对官方 `usd/` 树的副本做对应修改（原文件不动，副本可随时重新生成）：

| 问题 | 修正 |
|---|---|
| `physics.usda` 中的 `root_joint` 是把 `base_link` 焊死在世界的 FixedJoint | 删除该关节定义 |
| ArticulationRoot 挂在 `root_joint` 上，删关节后失效 | 把 `PhysicsArticulationRootAPI` + `PhysxArticulationAPI` 移到机器人根 Xform（与官方 jetbot 资产同构） |
| 官方轮关节 drive 增益不可用：驱动轮 damping 太弱推不动整机，转向刚度折算过刚 | 在 `physx.usda` 中烘焙增益：转向 k=2 / d=0.2，驱动 k=0 / d=3，放开 maxForce 与限速 |

官方 USD 默认变体 `Robot=none` 只有骨架，加载时必须显式选择
`Robot=robot`、`Sensor=sensors`、`Physics=physx`。派生副本位于
`stages/galbot_s1_sim_src/`，是所有运行脚本的机器人来源。
`robots.py` 中的默认路径可用环境变量 `GALBOT_S1_USD` 覆盖。

场景组装脚本（`build_debug_stage.py`、`build_capture_stage.py`）用 USD 引用
组合仓库 + 机器人，并把变体选择和出生位姿写进 stage 文件。注意它们不定义
`PhysicsScene`——`World()` 会创建自己的物理场景，重复定义会报
"Physics scenes stepping is not the same"。

## Swerve 底盘运动学（swerve_controller.py）

底盘为 4 个舵轮模块，每个模块一个转向关节（revolute，绕 Z）+ 一个驱动关节
（continuous）。模块安装位置来自 USD 实测（`physics:localPos0`）：
`(±0.247, ±0.247)` m，约定 +x 前、+y 左、θ 绕 +z 逆时针为正（REP-103）。

`SwerveController.forward_with_state(command, current_steers)` 输入底盘 twist
`[vx, vy, wz]`，输出每个模块的 `(转向角 rad, 轮速 rad/s)`：

- 每轮速度 `v_i = (vx − wz·y_i, vy + wz·x_i)`，转向角为其辐角，轮速除以轮半径；
- 零速时保持当前转向角、轮速置零，避免转向电机无意义抖动；
- 目标角与当前角差超过 90° 时翻转 180° 并反转轮速，减少转向行程；
- 转向角限位 ±166.88°（URDF/USD 一致）。

实现是纯 NumPy，`tests/test_swerve_controller.py` 覆盖直行、后退、原地旋转、
定半径转弯、停止保持和翻转优化。

`wheel_radius=0.08 m` 是估计值；需要精确里程时应实测标定（见
[faq.md](faq.md)）。

## 双腕相机策略

机器人只有左右两路腕部 RGB-D 相机。设计上把"相机位置"和"相机方向"解耦：

- **位置**由臂姿决定：`configs/arm_pose.yaml` 的 `nav` 是左右严格镜像的紧凑
  导航姿态，腕部约在 base 前方 0.42 m、两侧 ±0.38 m，`torso_lift_joint1`
  控制整体高度。
- **方向**不做手臂 IK：Camera prim 挂在腕部 `*_arm_camera_link` 下跟随臂运动，
  双臂稳定后（约 400 个物理步 = 2 s @ 200 Hz），用
  `pose_utils.aimed_world_camera_quat(base_quat, pitch, yaw)` 相对机器人
  base 一次性对准两路相机。pitch/yaw 来自 `configs/wrist_cameras.yaml` 的
  `navigation_view`，roll 固定为 0，保证地平线水平。

这样避免了通过左右镜像的七关节链做方向 IK，也不会因左右 link 镜像轴与
Camera 轴约定的组合陷阱出错。

### 坐标轴约定

Isaac Sim 5 的 Camera API 有三套轴约定，不能混用（`pose_utils.py` 的核心主题）：

| `camera_axes` | 轴定义 | 用途 |
|---|---|---|
| `"world"` | +X 前、+Z 上 | 设置导航视线（机器人语义） |
| `"ros"` | +X 右、+Y 下、+Z 前 | OpenCV optical，导出外参 |
| `"usd"` | -Z 前、+Y 上 | USD Camera prim 原生约定 |

设置方向用 `"world"`；导出位姿直接读 `"ros"`，不需要再额外乘
`diag(1,-1,-1)` 手性翻转。

## GalbotS1Robot（robots.py）

`GalbotS1Robot` 实现 MobilityGen 的机器人协议：

- **`build()`**：引用派生 USD、设置变体、创建 `Articulation`，并在左右腕部
  `camera_link` 下定义 Camera prim，包成 `MobilityGenCamera` 挂入模块树——
  因此 replay 会自动渲染腕部通道。
- **`initialize()`**：物理就绪后按名称解析关节索引，不依赖 USD 排列顺序；
  可重入，未就绪时返回 False 由下一步重试。
- **`write_action()`**：action = `[v, wz]`，经 SwerveController 转换为
  转向位置目标 + 驱动速度目标；同时对臂身关节持续写 `arm_pose.yaml` 的
  保持目标，并触发一次性的腕部相机定向。
- **`update_state()`**：把基座位姿、关节状态写入 Buffer 并传播到子模块
  （相机），否则 RGB buffer 不会更新。
- **`write_replay_data()`**：replay 时恢复基座位姿和关节角；相机定向不属于
  关节状态，需按当前 base 朝向显式重建，否则画面会用 USD Camera 默认朝向。

底盘和占据图相关参数（占据半径 0.5 m、高度带 0.05–0.5 m、格子 0.05 m、
碰撞半径 0.45 m）、遥控增益和路径跟随参数均以类属性形式给出，可按需覆盖。

## 扩展机制（extension.py）

`galbot.mobility_gen` 扩展的启动顺序是关键：

1. `on_startup` 先 import `galbot.mobility_gen` 触发 `@ROBOTS.register()`；
2. 再程序化启用 `isaacsim.replicator.mobility_gen.ui`——UI 在自己的
   `on_startup` 里构建机器人下拉框，必须晚于注册，`GalbotS1Robot` 才会
   出现在列表里（所以 extension.toml 不声明对 UI 的依赖）；
3. 用保存绝对资产路径的版本替换 UI 模块命名空间里的 `save_stage` 绑定。

第 3 步的原因：官方 `save_stage` 把依赖路径相对于临时缓存层重写，
MobilityGen 再把该层复制进录制目录，replay（尤其通过软链接选择录制时）
路径会解析到错误位置，表现为场景缺失、RGB 全黑。替换后录制保存绝对路径，
可在任意目录下 replay。

修改扩展代码后必须完全重启 Isaac Sim；扩展在模块加载时读取 YAML 配置，
UI 内 Reset 不会重新读取。
