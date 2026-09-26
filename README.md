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
| 外部控制接口 | `[v, wz]` 速度指令 + 2D 位姿反馈；完整 ROS2 导航见相邻的 `../navigation` 与 `../sim_adapter` |

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

## 快速开始：在自己的场景中跑起来

目标是在一套新的 Isaac Sim 环境中，用你自己的场景驱动 Galbot S1。
扩展通过 `--ext-folder` 加载，不需要安装进 Isaac Sim 目录。完整操作细节见
[docs/workflows.md](docs/workflows.md)，与导航系统对接见
[docs/navigation.md](docs/navigation.md)。

```bash
cd "$GALBOT_WORKSPACE"

# 1. 初始化子模块（首次克隆后）
git submodule update --init --recursive

# 2. 生成可移动仿真资产
python3 scripts/make_sim_variant.py

# 3. 把采集场景指向你自己的仓库 USD：
#    修改 scripts/build_capture_stage.py 顶部的 WAREHOUSE 常量后执行
#    （可选参数：出生位姿 x y yaw_deg）
python3 scripts/build_capture_stage.py
```

然后在 GUI 中用自己的场景验证移动：

```bash
cd "$ISAAC_SIM"
./isaac-sim.sh \
  --ext-folder "$GALBOT_WORKSPACE/exts" \
  --enable galbot.mobility_gen
```

打开你的场景 USD → MobilityGen 面板 → 选择场景 USD、占据图 `map.yaml`、
Robot 选 `GalbotS1Robot`、Scenario 选 `KeyboardTeleoperation` → 点击
**Build**（等待约 2 秒双臂和相机视线稳定）→ `W/S` 前后移动、`A/D` 原地转向。

无头模式验证随机路径导航（先把 `scripts/test_mobility_gen_galbot.py` 中的
`WAREHOUSE`/`OMAP` 常量指向你的场景与占据图）：

```bash
MG_STEPS=2000 ISP scripts/test_mobility_gen_galbot.py \
  --enable isaacsim.replicator.mobility_gen.examples
```

底盘、相机等逐级验收脚本和数据采集属于进阶流程，不是运行前置条件，
需要时见 [docs/workflows.md](docs/workflows.md#2-逐级验收可选)。

### 无头自动 RGB-D 采集（推荐）

`capture_auto_rgbd.py` 是独立的无头采集入口：它复用当前 `GalbotS1Robot` 的
`cmd_vel → PlanarBaseController → Articulation` 理想底盘后端，不直接设置路径点
或底盘坐标。默认 `internal` 模式会从占据图采样可达目标、A* 规划并沿扫掠路径
检查碰撞；接入导航栈时可用 `cmd_vel` 模式，二者互斥。

```bash
cd "$GALBOT_WORKSPACE"
ISP scripts/capture_auto_rgbd.py \
  --scene /path/to/scene.usd \
  --map /path/to/map.yaml \
  --output "$GALBOT_WORKSPACE/outputs/galbot_rgbd" \
  --seed 7 --episodes 2 --frames 40 --sample-rate 5 \
  --headless --motion-source internal \
  --ext-folder "$GALBOT_WORKSPACE/exts" \
  --enable galbot.mobility_gen \
  --enable isaacsim.replicator.mobility_gen.examples
```

先做确定性 smoke test（只写 20 帧，不覆盖已有目录）：

```bash
ISP scripts/capture_auto_rgbd.py --scene /path/to/scene.usd \
  --map /path/to/map.yaml --output "$GALBOT_WORKSPACE/outputs/galbot_smoke" \
  --seed 7 --smoke --headless \
  --ext-folder "$GALBOT_WORKSPACE/exts" \
  --enable galbot.mobility_gen \
  --enable isaacsim.replicator.mobility_gen.examples
```

每个 episode 输出左右腕部 `rgb/*.png`、米制 float32 `depth_raw/*.npy`、
`depth_valid_mask/*.png`、毫米 uint16 `depth/*.png`、`camera_info.json`、
`pose.txt` 和逐帧 `manifest.jsonl`；根目录
有 `validation_report.json`。正式采集使用持久化输出目录；已有非空目录不会被覆盖，
脚本会自动创建带时间戳的子目录，需要复用固定目录时才加 `--overwrite`。完整输出字段和 `cmd_vel` profile 接法见
[docs/workflows.md](docs/workflows.md#3-无头自动-rgb-d-采集) 与
[docs/data_format.md](docs/data_format.md#无头自动-rgb-d-输出)。
当前扫描工厂场景的作业区、无头长序列和质量限制见
[docs/scene_with_collision_expanded.md](docs/scene_with_collision_expanded.md)。

轨迹可视化：

```bash
python3 scripts/plot_trajectory.py \
  --input "$GALBOT_WORKSPACE/outputs/full_warehouse_smoke_20260924" \
  --map /root/gpufree-data/data_usd/map_test.yaml \
  --output "$GALBOT_WORKSPACE/outputs/full_warehouse_smoke_20260924/trajectory.png"
```

## 文档索引

| 文档 | 内容 |
|---|---|
| [docs/navigation.md](docs/navigation.md) | 与导航栈对接：cmd_vel 接口、里程计反馈、目标点导航两条路径 |
| [`../sim_adapter/docs/operation.md`](../sim_adapter/docs/operation.md) | 已实现的 ROS2 导航启动、RViz 选点和 Isaac 适配流程 |
| [docs/architecture.md](docs/architecture.md) | 实现思路：资产派生、swerve 运动学、相机定向、扩展机制 |
| [docs/workflows.md](docs/workflows.md) | 操作指南：资产准备、逐级验收、数据采集、MobilityGen、replay |
| [docs/data_format.md](docs/data_format.md) | 输出数据格式与坐标约定 |
| [docs/scene_with_collision_expanded.md](docs/scene_with_collision_expanded.md) | 扫描工厂场景的局部地图、连续 RGB-D 采集与已验证范围 |
| [docs/configuration.md](docs/configuration.md) | 配置文件说明、场景与路径自定义方法 |
| [docs/faq.md](docs/faq.md) | 常见问题与已知限制 |
