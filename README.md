# Galbot S1 × Isaac Sim 5.0 × MobilityGen

本工程用于在仓库场景中驱动 Galbot S1，并从左右腕部 RGB-D 相机同步采集导航数据。
当前有效链路是：

```text
官方 Galbot 资产
  → 可移动的仿真派生资产
  → 四轮 swerve 底盘
  → 左右对称导航臂姿
  → 双腕 RGB-D（前视、下俯 10°、零滚转）
  → RGB / Z-depth / 内参 / OpenCV 世界位姿
```

机器人上只创建左右腕部两路相机。早期位于机身 `base_link/mast_camera` 下的 Hawk
虚拟双目相机已经移除。

## 1. 当前状态

| 模块 | 状态 | 说明 |
|---|---|---|
| Galbot USD | 可用 | 使用派生资产，不修改官方文件 |
| 浮动底盘 | 可用 | 已移除官方固定世界的 `root_joint` |
| Swerve 控制 | 可用 | 四轮独立转向和驱动 |
| 双腕相机 | 可用 | 仅保留 left/right wrist camera |
| 相机方向 | 已验证 | 高约 1.18 m，俯角约 -10°，左右无滚转 |
| 独立同步采集 | 可用 | RGB、深度、内参、逐帧位姿 |
| MobilityGen | 可用 | 注册、随机路径、录制和 replay |

相机坐标约定是本工程的重要前提：

- Isaac `camera_axes="world"`：`+X` 前、`+Z` 上，用于设置导航视线；
- OpenCV optical / Isaac `camera_axes="ros"`：`+X` 右、`+Y` 下、`+Z` 前，用于导出外参；
- USD Camera 原生坐标：`-Z` 前、`+Y` 上，不应与前两者混用。

## 2. 环境与路径

已验证环境：Ubuntu 22.04、RTX 4090、Isaac Sim 5.0。

```bash
export GALBOT_WORKSPACE=/root/gpufree-data/galbot_ws
export ISAAC_PYTHON=/root/isaacsim/python.sh
cd "$GALBOT_WORKSPACE"

# 可选：当前终端的简写
alias ISP='PYTHONUNBUFFERED=1 /root/isaacsim/python.sh'
```

路径依赖：

| 内容 | 路径 |
|---|---|
| 工程根目录 | `/root/gpufree-data/galbot_ws` |
| Isaac Sim Python | `/root/isaacsim/python.sh` |
| 仓库场景 | `/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/warehouse_with_forklifts_edit.usd` |
| 占据地图 | `/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/MobilityGenData/map.yaml` |
| MobilityGen 数据 | `/root/MobilityGenData`（数据盘软链接） |

带 `SimulationApp` 的脚本必须使用 `ISP` 或 `$ISAAC_PYTHON`，不能直接使用系统
`python3`。纯配置检查、资产生成和离线测试可以使用系统 Python。

## 3. 仓库结构

```text
galbot_ws/
├── configs/
│   ├── arm_pose.yaml                 # 导航臂姿和默认姿态
│   └── wrist_cameras.yaml            # 分辨率、深度范围、导航 pitch/yaw
├── exts/galbot.mobility_gen/
│   └── galbot/mobility_gen/
│       ├── extension.py              # 注册扩展并启用 MobilityGen UI
│       ├── robots.py                 # GalbotS1Robot、底盘和双腕相机
│       ├── swerve_controller.py      # 纯 NumPy swerve 运动学
│       └── pose_utils.py             # 相机坐标系和位姿工具
├── scripts/
│   ├── inspect_galbot_s1.py          # 检查 USD
│   ├── make_sim_variant.py           # 生成可移动机器人派生资产
│   ├── build_debug_stage.py          # 构建机器人调试场景
│   ├── build_capture_stage.py        # 构建仓库采集场景
│   ├── test_galbot_motion.py         # 底盘物理验收
│   ├── preview_arm_pose.py           # 双腕导航视野预览
│   ├── test_wrist_cameras.py         # RGB-D / K / pose 独立验收
│   ├── capture_wrist_dataset.py      # 行驶中同步采集
│   └── test_mobility_gen_galbot.py   # MobilityGen 无头集成测试
├── stages/
│   ├── galbot_s1_sim_src/            # 由官方资产生成的可移动派生副本
│   ├── galbot_s1_debug.usd
│   ├── galbot_warehouse_capture.usd
│   └── galbot_s1_inspection.md
├── tests/
│   ├── test_camera_pose.py            # 相机轴约定离线测试
│   └── test_swerve_controller.py      # 底盘运动学离线测试
├── outputs/
│   └── arm_pose_preview/              # 当前保留的正确 nav 验收图
├── third_party/galbot_s1_description/ # Galbot 官方资产，只读
└── README.md
```

不要直接编辑 `third_party/galbot_s1_description`。需要改变物理结构时修改生成脚本，
然后重建 `stages/galbot_s1_sim_src`。

## 4. 首次准备或资产更新

正常使用已有 stage 时可以跳过本节。只有官方资产更新、派生目录缺失或需要改变出生点时
才需要重建。

```bash
cd "$GALBOT_WORKSPACE"

# 1. 检查官方资产；报告写入 stages/galbot_s1_inspection.md
python3 scripts/inspect_galbot_s1.py

# 2. 生成可移动仿真副本
python3 scripts/make_sim_variant.py

# 3. 生成调试场景和仓库采集场景
python3 scripts/build_debug_stage.py
python3 scripts/build_capture_stage.py
```

自定义仓库出生位姿：

```bash
# x=2.0 m, y=-8.0 m, yaw=90°
python3 scripts/build_capture_stage.py 2.0 -8.0 90
```

派生资产完成三项必要修正：删除固定世界关节、把 ArticulationRoot 放到机器人根节点、
写入可用的转向/驱动增益和速度上限。

## 5. 推荐工作流

### 5.1 先运行离线测试

无需启动 Isaac Sim：

```bash
python3 tests/test_camera_pose.py
python3 tests/test_swerve_controller.py
```

两项均应无异常退出。

### 5.2 验证底盘物理

```bash
ISP scripts/test_galbot_motion.py
```

脚本检查静止、直行、后退、原地旋转、定半径转弯、停止和站立稳定。默认验收标准包括
直行约 `0.9 m @ 0.3 m/s`，原地旋转漂移小于约 `7 cm`。

### 5.3 预览和调整双腕视角

当前默认值：

```yaml
# configs/wrist_cameras.yaml
navigation_view:
  pitch_deg: -10.0  # 负值向下
  yaw_deg: 0.0      # 正值向左
```

预览当前配置：

```bash
ISP scripts/preview_arm_pose.py
```

临时尝试其他角度，不写配置：

```bash
ISP scripts/preview_arm_pose.py --pitch -15 --yaw 0
```

确认画面后保存：

```bash
ISP scripts/preview_arm_pose.py --pitch -15 --yaw 0 --save-view
```

输出：

```text
outputs/arm_pose_preview/nav_left_wrist.png
outputs/arm_pose_preview/nav_right_wrist.png
```

控制台会同时打印相机高度、实测俯角、世界前向向量和深度中位数。两路相机应满足：

- 高度接近；
- pitch 接近配置值；
- forward 基本一致；
- 地平线水平，没有左右滚转；
- 能同时看到前方障碍物和部分近处地面。

`configs/arm_pose.yaml` 只负责相机位置。相机方向会在机械臂稳定后直接相对机器人
base 对准，不再通过七关节 IK 间接求方向。修改 YAML 后需重启已运行的 Isaac Sim。

### 5.4 独立验证相机

```bash
ISP scripts/test_wrist_cameras.py
```

脚本使用默认 `nav` 臂姿，等待双臂稳定，然后采集 6 帧。输出位于：

```text
outputs/wrist_camera_test/
├── left_wrist_camera_info.json
├── left_wrist_rgb_000.png
├── left_wrist_depth_000.npy
├── left_wrist_depth16_000.png
├── left_wrist_pose_000.txt
└── right_wrist_...                  # 对称文件
```

检查标准：RGB 非黑、深度存在有效值、两路俯角接近、`K` 的主点接近图像中心、pose
为 `world_from_camera_opencv` 的合法 4×4 刚体变换。

### 5.5 采集导航数据

```bash
ISP scripts/capture_wrist_dataset.py \
  --motion straight --frames 40 --speed 0.25
```

参数：

| 参数 | 可选值/含义 | 默认值 |
|---|---|---|
| `--motion` | `straight`、`spin`、`square` | `straight` |
| `--frames` | 保存帧数 | `48` |
| `--speed` | 直线速度，m/s | `0.25` |
| `--scene-name` | 指定输出目录名 | 自动 `scene_NNN` |

每帧执行顺序为：物理推进 → 渲染 → 读取同一帧 RGB/深度/位姿 → 写盘。采样频率约
5 Hz。输出结构：

```text
outputs/scene_NNN/
├── left_wrist/
│   ├── rgb/00000.png                # uint8 RGB
│   ├── depth/00000.png              # uint16，毫米
│   ├── depth_raw/00000.npy          # float32，米，Z-depth
│   ├── camera_info.json             # 实际渲染内参 K
│   ├── manifest.jsonl               # 帧号、时间、底盘/轮子/深度统计
│   └── pose.txt                     # 每帧一个 4×4 world_from_camera_opencv
└── right_wrist/                     # 同结构
```

权威深度是 `depth_raw/*.npy`；PNG 主要用于通用工具读取和快速查看。

## 6. MobilityGen

### 6.1 无头集成测试

```bash
MG_STEPS=2000 ISP scripts/test_mobility_gen_galbot.py \
  --enable isaacsim.replicator.mobility_gen.examples
```

该测试执行机器人注册、仓库构建、随机路径跟踪和状态录制。正常情况下累计里程应大于
1 m。测试输出写入：

```text
/root/MobilityGenData/recordings/galbot_s1_headless_test/
```

这是可删除、可重建的测试目录，不要与正式时间戳录制混用。少于约 2000 步的短测试可能
能证明构建成功，但会因为里程不足触发脚本末尾的 `>1 m` 断言。

### 6.2 GUI 键盘遥控和录制

```bash
cd /root/isaacsim
./isaac-sim.sh \
  --ext-folder /root/gpufree-data/galbot_ws/exts \
  --enable galbot.mobility_gen
```

1. 打开 `warehouse_with_forklifts_edit.usd`。
2. 打开 MobilityGen 面板。
3. Scene USD 选择仓库 USD。
4. Occupancy Map 选择 `MobilityGenData/map.yaml`。
5. Robot 选择 `GalbotS1Robot`，Scenario 选择 `KeyboardTeleoperation`。
6. 点击 **Build**，等待约 2 秒让双臂和相机视线稳定。
7. 使用 `W/S` 前后移动，`A/D` 原地转向。
8. 点击 **Record** 开始录制。

新构建的机器人只有两路腕部相机：

```text
robot.left_wrist_camera
robot.right_wrist_camera
```

不存在 `robot.front_camera.left/right`。若界面里仍看到旧的机身 Hawk 相机，请完全重启
Isaac Sim 并重新 Build；旧 stage 实例不会被热更新自动删除。

### 6.3 Replay 渲染

先运行 6.1 的无头集成测试生成完整录制，并确认初始化文件存在：

```bash
test -f /root/MobilityGenData/recordings/galbot_s1_headless_test/config.json
test -f /root/MobilityGenData/recordings/galbot_s1_headless_test/stage.usd
test -f /root/MobilityGenData/recordings/galbot_s1_headless_test/occupancy_map/map.yaml
```

上述命令均无输出且退出码为 0 后，再执行：

`replay_directory.py` 的 `--input` 必须指向“包含一个或多个录制目录的父目录”，不能
直接指向某一次录制。否则脚本会把该录制内的 `occupancy_map/`、`state/` 等误认为独立
录制，并尝试读取 `occupancy_map/config.json`。下面使用软链接建立临时输入集合：它不会
复制数据，删除链接也不会删除原始录制，并且可以只选择需要 replay 的记录。也可以直接
传入 `/root/MobilityGenData/recordings`，但这样会处理其中的全部录制，包括可能残缺的目录。

```bash
mkdir -p /tmp/replays_in
ln -sfn /root/MobilityGenData/recordings/galbot_s1_headless_test \
  /tmp/replays_in/galbot_s1_headless_test

cd /root/isaacsim
./python.sh standalone_examples/replicator/mobility_gen/replay_directory.py \
  --input /tmp/replays_in \
  --render_interval 20 \
  --depth_enabled True \
  --ext-folder /root/gpufree-data/galbot_ws/exts \
  --enable galbot.mobility_gen \
  --enable isaacsim.replicator.mobility_gen.examples
```

结果位于 `/root/MobilityGenData/replays/galbot_s1_headless_test/`，只包含双腕相机通道。

Galbot 扩展会覆盖 MobilityGen UI 的临时 stage 保存函数，将场景依赖保存为绝对路径。
这是必要的：官方实现先相对于 `/tmp` 重写依赖，再把 stage 复制到录制目录，可能产生
`../../root/...`；通过其他目录或软链接 replay 时，该路径会错误解析为 `/tmp/root/...`，
最终表现为仓库场景缺失、RGB 全黑。修改扩展后必须完全重启 Isaac Sim，已创建的旧录制
不会被自动修复。

旧录制可保留备份后，用 `config.json` 中的原始 `scene_usd` 替换损坏的 stage。例如：

```bash
REC=/root/MobilityGenData/recordings/2026-09-20_17-04-02-645520
SCENE=/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/warehouse_with_forklifts_edit.usd
cp -p "$REC/stage.usd" "$REC/stage.usd.broken.bak"
cp "$SCENE" "$REC/stage.usd"
```

## 7. 配置说明

### `configs/arm_pose.yaml`

- `default_pose`：采集和 MobilityGen 默认使用的姿态名；
- `nav`：左右严格镜像的紧凑导航姿态；
- 未列出的关节按 `0` 处理；
- `torso_lift_joint1` 控制整体相机高度。

不要为了改变俯角而修改手臂关节；俯角应在 `wrist_cameras.yaml` 中设置。

### `configs/wrist_cameras.yaml`

- `navigation_view`：当前代码实际读取的相对机器人 base 的 pitch/yaw；
- `fx/fy/cx/cy: null`：表示内参由 Isaac Sim 渲染相机实时读取；
- `width/height`、`near_m/far_m`、`depth_scale`：为后续实机/统一相机工厂预留；
  当前三个脚本固定使用 `640×480`、Isaac 默认裁剪范围和 `1000 mm/m`；
- `calibration_source: placeholder`：提醒该 YAML 不是实机标定结果。采集输出中的
  `camera_info.json` 会标记为 `isaacsim_render`。

真机部署前必须用真实相机内参、畸变参数和手眼外参替换占位值。

## 8. 常见问题

### 画面朝地或左右旋转

确认没有重新引入硬编码腕部四元数，并使用当前 `pose_utils.py`。设置方向应使用
`camera_axes="world"`，导出 OpenCV 外参应直接读取 `camera_axes="ros"`，不要再乘一次
`diag(1,-1,-1)`。

### 相机不跟随机器人

Camera prim 必须位于对应腕部刚体路径下：

```text
/World/robot/left_arm_link7/left_arm_camera_link/sensors/rgbd_camera
/World/robot/right_arm_link7/right_arm_camera_link/sensors/rgbd_camera
```

### 修改配置后没有变化

独立脚本每次启动都会重新读取 YAML。MobilityGen 扩展在模块加载时读取配置，因此必须
完全重启 Isaac Sim，不能只在 UI 中 Reset。

### MobilityGen Build 后机器人不动

确认使用 `stages/galbot_s1_sim_src/galbot_s1.usda`，而不是官方固定基座 USD；同时确认
变体选择为 `Robot=robot`、`Sensor=sensors`、`Physics=physx`。

### 仓库里实际速度偏低

仓库地面存在轮胎打滑，当前 `wheel_radius=0.08 m` 也是估计值。需要精确里程时应先标定
轮半径和地面/轮胎摩擦参数。

## 9. 已知限制

- `wrist_cameras.yaml` 的实机标定字段仍是占位值；
- 尚未完成平墙深度误差和 `K + pose` 重投影误差测试；
- `square` 运动模式是简单时序控制，不是闭环方形轨迹；
- MobilityGen 测试会固定写入 `galbot_s1_headless_test`，正式数据应使用时间戳目录；
- replay 的深度可视化 PNG 不能替代原始浮点深度。

## 10. 常用命令

```bash
cd /root/gpufree-data/galbot_ws
alias ISP='PYTHONUNBUFFERED=1 /root/isaacsim/python.sh'

# 离线测试
python3 tests/test_camera_pose.py
python3 tests/test_swerve_controller.py

# 相机预览和验证
ISP scripts/preview_arm_pose.py
ISP scripts/test_wrist_cameras.py

# 采集
ISP scripts/capture_wrist_dataset.py --motion straight --frames 40 --speed 0.25

# 底盘和 MobilityGen
ISP scripts/test_galbot_motion.py
MG_STEPS=2000 ISP scripts/test_mobility_gen_galbot.py \
  --enable isaacsim.replicator.mobility_gen.examples
```
