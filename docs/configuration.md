# 配置说明与场景自定义

## 配置文件

### configs/arm_pose.yaml

- `default_pose`：采集和 MobilityGen 默认使用的姿态名；
- `nav`：左右严格镜像的紧凑导航姿态，腕部约位于 base 前方 0.42 m、
  两侧 ±0.38 m；
- 未列出的关节按 `0` 处理；
- `torso_lift_joint1` 控制整体相机高度。

不要为了改变俯角而修改手臂关节；俯角在 `wrist_cameras.yaml` 的
`navigation_view` 中设置。

### configs/wrist_cameras.yaml

- `navigation_view`：导航视线的 pitch/yaw（相对机器人 base），
  负 pitch 向下、正 yaw 向左、roll 固定为 0；
- `left_wrist`/`right_wrist` 的 `fx/fy/cx/cy: null`：内参由 Isaac Sim 渲染
  相机实时读取，实际值记录在输出的 `camera_info.json`；
- `width/height`、`near_m/far_m`、`depth_scale`：为实机/统一相机工厂预留，
  当前脚本固定使用 `640×480`、Isaac 默认裁剪范围和 `1000 mm/m`；
- `calibration_source: placeholder`：标记该 YAML 不是实机标定结果。

真机部署前必须用真实相机内参、畸变参数和手眼外参替换占位值。

配置的读取时机：独立脚本每次启动重新读取 YAML；MobilityGen 扩展在模块
加载时读取，修改后需完全重启 Isaac Sim。

## 场景与路径自定义

本仓库不假设场景资产的位置，需要用户按环境指定。分三种情况：

### 机器人 USD

`robots.py` 默认使用 `<仓库>/stages/galbot_s1_sim_src/galbot_s1.usda`，
可用环境变量覆盖：

```bash
export GALBOT_S1_USD=/path/to/your/galbot_s1.usda
```

### 仓库场景与占据图

以下脚本通过文件顶部的常量引用外部场景资产，换场景时修改对应常量：

| 脚本 | 常量 | 含义 |
|---|---|---|
| `scripts/build_capture_stage.py` | `WAREHOUSE` | 组合采集场景引用的仓库 USD |
| `scripts/test_mobility_gen_galbot.py` | `WAREHOUSE`、`OMAP` | 无头测试的仓库 USD 与占据图 `map.yaml` |

示例（`build_capture_stage.py`）：

```python
WAREHOUSE = "/path/to/your/warehouse.usd"
```

修改后重新运行 `python3 scripts/build_capture_stage.py [x y yaw_deg]`
重建组合场景。GUI 流程在 MobilityGen 面板中直接选择自己的场景 USD 和
占据图，无需改代码。

占据地图 `map.yaml` 是 ROS 格式的地图元数据，可由 MobilityGen UI 在
打开场景后生成，或使用其他建图工具产物；需要与场景 USD 的坐标系一致。

### 数据输出目录

- 独立采集输出固定在 `<仓库>/outputs/`（`.gitignore` 只保留验收预览图，
  数据集不入库）；
- MobilityGen 录制与 replay 固定在 `~/MobilityGenData/` 下的
  `recordings/` 与 `replays/`。

### Isaac Sim 路径

所有文档命令使用 `ISAAC_SIM` 环境变量指向安装目录，`ISP` 别名调用其
`python.sh`。首次使用时：

```bash
export ISAAC_SIM=/path/to/isaacsim
alias ISP='PYTHONUNBUFFERED=1 "$ISAAC_SIM/python.sh"'
```

## 出生位姿

`build_capture_stage.py` 的命令行参数即出生位姿（x、y 米，yaw 度）：

```bash
python3 scripts/build_capture_stage.py 2.0 -8.0 90   # x=2, y=-8, yaw=90°
```

出生点应选在场景中无碰撞、且占据图上的可行区域。MobilityGen GUI 构建时
会从占据图随机采样或手动指定初始位姿，不依赖该默认值。
