# 输出数据格式与坐标约定

## 坐标约定

所有输出使用以下约定（详见 [architecture.md](architecture.md#坐标轴约定)）：

- 相机外参为 OpenCV optical 约定（即 Isaac `camera_axes="ros"`）：
  +X 右、+Y 下、+Z 前；
- 四元数为 scalar-first（wxyz）；
- 深度为 `distance_to_image_plane`，即沿光轴的 Z-depth，单位米。

三套相机轴约定的对照（设置视线用 `"world"`，导出用 `"ros"`，不要用 USD
原生约定导出）：

| `camera_axes` | 轴定义 |
|---|---|
| `"world"` | +X 前、+Z 上 |
| `"ros"` | +X 右、+Y 下、+Z 前 |
| `"usd"` | -Z 前、+Y 上 |

## 独立采集输出

`scripts/capture_wrist_dataset.py` 与 `scripts/test_wrist_cameras.py` 的输出：

```text
outputs/scene_NNN/                     # 或 outputs/wrist_camera_test/
├── left_wrist/
│   ├── rgb/00000.png                  # uint8 RGB
│   ├── depth/00000.png                # uint16，毫米
│   ├── depth_raw/00000.npy            # float32，米，Z-depth（权威深度）
│   ├── camera_info.json               # 实际渲染内参
│   ├── manifest.jsonl                 # 每帧元数据
│   └── pose.txt                       # 每帧一个 4×4 外参
└── right_wrist/                       # 同结构
```

采样频率约 5 Hz（每 12 个物理步采 1 帧）。帧内顺序为物理推进 → 渲染 →
读取同一帧 RGB/深度/位姿 → 写盘，保证多模态严格同步。

### camera_info.json

```json
{
  "width": 640,
  "height": 480,
  "K": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "model": "plumb_bob",
  "D": [0.0, 0.0, 0.0, 0.0, 0.0],
  "depth_scale": 1000.0,
  "depth_units": "distance_to_image_plane(z, meters)",
  "calibration_source": "isaacsim_render"
}
```

`K` 从 Isaac Sim 渲染相机实时读取，是实际生效的内参；`D` 全零表示仿真
针孔模型无畸变。`calibration_source` 标记数据来源，实机标定后应替换。

### manifest.jsonl

每行一帧：

```json
{
  "frame": 0,
  "sim_time": 2.0,
  "base_pose_xyz": [0.5, -5.0, 0.05],
  "wheel_steer": [0.0, 0.0, 0.0, 0.0],
  "wheel_drive": [0.0, 0.0, 0.0, 0.0],
  "camera_pos": [0.9, -5.4, 1.18],
  "depth_min": 0.8,
  "depth_p50": 4.2,
  "rgb_mean": 103.5
}
```

字段含义：仿真时间（秒，物理步累计）、机器人基座世界坐标、四轮转向角
（rad）与驱动角（rad）、相机世界坐标、有效深度的最小值与中位数（米）、
RGB 均值。深度统计只计入有限值且小于 100 m 的像素。

### pose.txt

每帧一段：`# frame N` 注释行 + 4×4 矩阵（每行 6 位小数）。矩阵为
`world_from_camera_opencv`，把 OpenCV 相机系下的点变换到世界系。

### 深度格式选择

`depth_raw/*.npy`（float32，米）是权威深度；`depth/*.png`（uint16，毫米）
为通用工具兼容的换算版本，存在量化误差。做定量分析时始终使用前者。

相机独立验收（`test_wrist_cameras.py`）使用略不同的文件命名
（`left_wrist_rgb_000.png`、`left_wrist_depth_000.npy`、
`left_wrist_depth16_000.png`、`left_wrist_pose_000.txt`、
`left_wrist_camera_info.json`），内容语义相同。

## MobilityGen 录制

录制目录位于 `~/MobilityGenData/recordings/`，一个完整录制包含：

```text
<录制目录>/
├── config.json            # 场景/机器人/场景类型配置，含 scene_usd 原始路径
├── stage.usd              # 构建时场景快照（本扩展保存为绝对资产路径）
├── occupancy_map/map.yaml # 占据地图
└── state/                 # 逐步状态帧（由 MobilityGenWriter 写入）
```

replay 结果写入 `~/MobilityGenData/replays/<录制目录名>/`，只包含双腕相机
通道；其深度 PNG 仅供快速查看，定量使用请回原始录制读取状态帧。

无头集成测试固定写入 `galbot_s1_headless_test/` 目录；正式数据使用
GUI 录制的时间戳目录。
