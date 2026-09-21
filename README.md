# Galbot S1 × Isaac Sim 5.0 × MobilityGen

本工程用于在仓库场景中驱动 Galbot S1，并从左右腕部 RGB-D 相机同步采集导航数据。
整体链路：

```text
官方 Galbot 资产
  → 可移动的仿真派生资产
  → 四轮 swerve 底盘
  → 左右对称导航臂姿
  → 双腕 RGB-D（前视、下俯 10°、零滚转）
  → RGB / Z-depth / 内参 / OpenCV 世界位姿
  → MobilityGen 随机路径、录制与 replay
```

机器人只提供左右两路腕部相机，通过 `robot.left_wrist_camera` /
`robot.right_wrist_camera` 访问。

## 功能概览

| 模块 | 说明 |
|---|---|
| 仿真资产 | 由官方 USD 派生的可移动副本，不修改官方文件 |
| Swerve 底盘 | 四轮独立转向和驱动，纯 NumPy 运动学，可离线测试 |
| 双腕相机 | 640×480 RGB-D，臂姿决定位置，视线相对 base 对准 |
| 独立同步采集 | RGB、深度、实际渲染内参、逐帧 OpenCV 位姿 |
| MobilityGen | 机器人注册、键盘遥控、随机路径、录制和 replay |

## 环境要求

- Ubuntu 22.04 + RTX GPU（已在 RTX 4090 验证），Isaac Sim 5.0
- 系统 Python 3.10+：`pip install usd-core PyYAML numpy pillow`
  （离线资产生成、配置检查、单元测试使用）
- Isaac Sim 自带 Python（`SimulationApp` 脚本必须使用它，不能用系统 `python3`）

设置环境变量（建议写入 `~/.bashrc`）：

```bash
# 本仓库的克隆位置
export GALBOT_WORKSPACE=/path/to/galbot_ws
# Isaac Sim 安装目录
export ISAAC_SIM=/path/to/isaacsim

# 运行 Isaac Sim 脚本的简写（带无缓冲输出）
alias ISP='PYTHONUNBUFFERED=1 "$ISAAC_SIM/python.sh"'
```

仓库场景 USD、占据地图等外部资产路径在脚本中通过常量定义，
替换成自己的场景时参见 [docs/configuration.md](docs/configuration.md)。

## 目录结构

```text
galbot_ws/
├── configs/                            # 臂姿与相机配置（YAML）
├── docs/                               # 详细文档（见下方索引）
├── exts/galbot.mobility_gen/           # Isaac Sim 扩展：机器人、底盘、相机
├── scripts/                            # 资产生成、验证、采集脚本
├── stages/
│   ├── galbot_s1_sim_src/              # 由官方资产生成的可移动派生副本
│   ├── galbot_s1_debug.usd             # 机器人调试场景
│   └── galbot_warehouse_capture.usd    # 仓库 + 机器人组合采集场景
├── tests/                              # 离线单元测试（无需 Isaac Sim）
├── outputs/                            # 采集与预览输出（大部分不入库）
└── third_party/galbot_s1_description/  # Galbot 官方资产子模块，只读
```

不要直接编辑 `third_party/galbot_s1_description`。需要改变物理结构时修改
`scripts/make_sim_variant.py`，然后重建 `stages/galbot_s1_sim_src`。

## 快速开始

按顺序执行；每一步的细节和参数见 [docs/workflows.md](docs/workflows.md)。

```bash
cd "$GALBOT_WORKSPACE"

# 1. 初始化子模块（首次克隆后）
git submodule update --init --recursive

# 2. 离线单元测试（相机轴约定、底盘运动学）
python3 tests/test_camera_pose.py
python3 tests/test_swerve_controller.py

# 3. 生成仿真资产与场景（官方资产更新或派生目录缺失时才需要）
python3 scripts/inspect_galbot_s1.py
python3 scripts/make_sim_variant.py
python3 scripts/build_debug_stage.py
python3 scripts/build_capture_stage.py

# 4. 底盘物理验收
ISP scripts/test_galbot_motion.py

# 5. 预览并调整双腕导航视野
ISP scripts/preview_arm_pose.py

# 6. 相机独立验收（RGB-D / 内参 / 位姿）
ISP scripts/test_wrist_cameras.py

# 7. 行驶中同步采集导航数据
ISP scripts/capture_wrist_dataset.py --motion straight --frames 40 --speed 0.25

# 8. MobilityGen 无头集成测试（随机路径 + 录制）
MG_STEPS=2000 ISP scripts/test_mobility_gen_galbot.py \
  --enable isaacsim.replicator.mobility_gen.examples
```

GUI 键盘遥控和 replay 渲染见 [docs/workflows.md](docs/workflows.md#mobilitygen)。

## 文档索引

| 文档 | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 实现思路：资产派生、swerve 运动学、相机定向、扩展机制 |
| [docs/workflows.md](docs/workflows.md) | 操作指南：资产准备、逐级验收、数据采集、MobilityGen、replay |
| [docs/data_format.md](docs/data_format.md) | 输出数据格式与坐标约定 |
| [docs/configuration.md](docs/configuration.md) | 配置文件说明、场景与路径自定义方法 |
| [docs/faq.md](docs/faq.md) | 常见问题与已知限制 |
