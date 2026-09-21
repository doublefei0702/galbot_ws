# 操作指南

环境变量 `GALBOT_WORKSPACE`、`ISAAC_SIM` 和 `ISP` 别名的设置见
[README](../README.md#环境要求)。

## 1. 资产准备

正常使用已有 stage 时可跳过本节。只有首次克隆、官方资产更新、派生目录缺失
或需要改变出生点时才需要执行。

```bash
cd "$GALBOT_WORKSPACE"

# 检查官方资产，报告写入 stages/galbot_s1_inspection.md
python3 scripts/inspect_galbot_s1.py

# 生成可移动仿真副本 stages/galbot_s1_sim_src/
python3 scripts/make_sim_variant.py

# 生成机器人调试场景和仓库采集场景
python3 scripts/build_debug_stage.py
python3 scripts/build_capture_stage.py
```

自定义仓库出生位姿（x、y 单位米，yaw 单位度）：

```bash
python3 scripts/build_capture_stage.py 2.0 -8.0 90
```

`build_capture_stage.py` 引用的仓库 USD 由脚本内 `WAREHOUSE` 常量决定，
替换场景的方法见 [configuration.md](configuration.md#场景与路径自定义)。

## 2. 逐级验收（可选）

本章脚本用于首次接入、修改代码后或排查问题时的分级验证，**不是运行前置
条件**——直接跑自己的场景见 [README 快速开始](../README.md#快速开始在自己的场景中跑起来)。
建议在改动底盘、相机或扩展代码后按顺序执行一遍。

### 2.1 离线单元测试

无需启动 Isaac Sim，两项均应无异常退出：

```bash
python3 tests/test_camera_pose.py
python3 tests/test_swerve_controller.py
```

### 2.2 底盘物理验收

```bash
ISP scripts/test_galbot_motion.py
```

脚本检查静止、直行、后退、原地旋转、定半径转弯、停止和站立稳定。默认验收
标准包括直行约 `0.9 m @ 0.3 m/s`、原地旋转漂移小于约 `7 cm`。可用
`--stage` 指定其他场景、`--wheel-radius` 覆盖轮半径。

### 2.3 预览和调整双腕视野

```bash
ISP scripts/preview_arm_pose.py                 # 使用当前配置
ISP scripts/preview_arm_pose.py --pitch -15     # 临时尝试，不写配置
ISP scripts/preview_arm_pose.py --pitch -15 --save-view   # 确认后保存
```

参数：`--poses` 选择 `arm_pose.yaml` 中的姿态（逗号分隔）、`--pitch`/`--yaw`
视线角度（pitch 负值向下，yaw 正值向左）、`--save-view` 把角度写回
`wrist_cameras.yaml`、`--stage` 指定预览场景。

输出双路画面到 `outputs/arm_pose_preview/`，控制台同时打印相机高度、实测
俯角、世界前向向量和深度中位数。合格标准：两路高度和俯角接近配置值、
forward 基本一致、地平线水平无滚转、能同时看到前方障碍物和近处地面。

### 2.4 相机独立验收

```bash
ISP scripts/test_wrist_cameras.py
```

使用默认 `nav` 臂姿，等待双臂稳定后采集 6 帧，输出到
`outputs/wrist_camera_test/`（文件结构见
[data_format.md](data_format.md#独立采集输出)）。检查标准：RGB 非黑、深度
存在有效值、两路俯角接近、`K` 的主点接近图像中心、pose 为合法 4×4
`world_from_camera_opencv` 刚体变换。

## 3. 数据采集

```bash
ISP scripts/capture_wrist_dataset.py --motion straight --frames 40 --speed 0.25
```

| 参数 | 可选值/含义 | 默认值 |
|---|---|---|
| `--motion` | `straight`、`spin`、`square` | `straight` |
| `--frames` | 保存帧数 | `48` |
| `--speed` | 直线速度，m/s | `0.25` |
| `--scene-name` | 指定输出目录名 | 自动 `scene_NNN` |

采集流程：打开 `stages/galbot_warehouse_capture.usd` → 双臂稳定并定向相机 →
行驶中每 12 个物理步采 1 帧（约 5 Hz），帧内顺序为物理推进 → 渲染 → 同帧读取
RGB/深度/位姿 → 写盘。输出结构见 [data_format.md](data_format.md#独立采集输出)。

## 4. MobilityGen

### 4.1 无头集成测试

```bash
MG_STEPS=2000 ISP scripts/test_mobility_gen_galbot.py \
  --enable isaacsim.replicator.mobility_gen.examples
```

`MG_STEPS` 环境变量控制仿真步数（默认 2000）。测试执行机器人注册、仓库
构建、随机路径跟随和全程录制，末尾断言累计里程大于 1 m 并校验录制可被
reader 读取。录制写入 `~/MobilityGenData/recordings/galbot_s1_headless_test/`
（固定测试目录，可删除重建，不要与正式时间戳录制混用）。步数太少时里程
不足会触发断言失败。

### 4.2 GUI 键盘遥控和录制

```bash
cd "$ISAAC_SIM"
./isaac-sim.sh \
  --ext-folder "$GALBOT_WORKSPACE/exts" \
  --enable galbot.mobility_gen
```

1. 打开仓库场景 USD（自己的场景见
   [configuration.md](configuration.md#场景与路径自定义)）；
2. 打开 MobilityGen 面板；
3. Scene USD 选择仓库 USD，Occupancy Map 选择对应 `map.yaml`；
4. Robot 选择 `GalbotS1Robot`，Scenario 选择 `KeyboardTeleoperation`；
5. 点击 **Build**，等待约 2 秒让双臂和相机视线稳定；
6. `W/S` 前后移动，`A/D` 原地转向；
7. 点击 **Record** 开始录制，正式录制使用时间戳目录。

### 4.3 Replay 渲染

`replay_directory.py` 的 `--input` 必须指向**包含一个或多个录制目录的父
目录**，不能直接指向某一次录制——否则脚本会把录制内部的 `occupancy_map/`、
`state/` 等误认为独立录制。常用做法是用软链接挑选需要 replay 的录制：

```bash
mkdir -p /tmp/replays_in
ln -sfn ~/MobilityGenData/recordings/<录制目录名> /tmp/replays_in/<录制目录名>

cd "$ISAAC_SIM"
./python.sh standalone_examples/replicator/mobility_gen/replay_directory.py \
  --input /tmp/replays_in \
  --render_interval 20 \
  --depth_enabled True \
  --ext-folder "$GALBOT_WORKSPACE/exts" \
  --enable galbot.mobility_gen \
  --enable isaacsim.replicator.mobility_gen.examples
```

结果写入 `~/MobilityGenData/replays/<录制目录名>/`，只包含双腕相机通道。
本扩展保存的 stage 使用绝对资产路径，录制通过软链接或其他目录 replay 均
有效。replay 输出的深度 PNG 仅供快速查看，不能替代原始浮点深度。

## 5. 常用命令速查

```bash
# 离线测试
python3 tests/test_camera_pose.py
python3 tests/test_swerve_controller.py

# 相机预览和验证
ISP scripts/preview_arm_pose.py
ISP scripts/test_wrist_cameras.py

# 底盘与集成
ISP scripts/test_galbot_motion.py
MG_STEPS=2000 ISP scripts/test_mobility_gen_galbot.py \
  --enable isaacsim.replicator.mobility_gen.examples

# 采集
ISP scripts/capture_wrist_dataset.py --motion straight --frames 40 --speed 0.25
```
