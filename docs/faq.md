# 常见问题与已知限制

## 常见问题

### 画面朝地或左右旋转

相机方向必须按 [architecture.md](architecture.md#坐标轴约定) 的轴约定处理：
设置方向用 `camera_axes="world"`，导出 OpenCV 外参直接读 `camera_axes="ros"`，
不要再额外乘一次 `diag(1,-1,-1)` 手性翻转；也不要硬编码腕部四元数。

### 相机不跟随机器人

Camera prim 必须位于对应腕部刚体路径下：

```text
/World/robot/left_arm_link7/left_arm_camera_link/sensors/rgbd_camera
/World/robot/right_arm_link7/right_arm_camera_link/sensors/rgbd_camera
```

### 修改配置或扩展后没有变化

独立脚本每次启动都会重新读取 YAML。MobilityGen 扩展在模块加载时读取配置，
修改扩展代码或 YAML 后必须完全重启 Isaac Sim，只在 UI 中 Reset 无效。

### MobilityGen Build 后机器人不动

确认使用 `stages/galbot_s1_sim_src/galbot_s1.usda`（派生副本）而不是官方
固定基座 USD；同时确认变体选择为 `Robot=robot`、`Sensor=sensors`、
`Physics=physx`。原因见 [architecture.md](architecture.md#仿真资产派生make_sim_variantpy)。

### 仓库里实际速度偏低

仓库地面存在轮胎打滑，`wheel_radius=0.08 m` 也是估计值。需要精确里程时
应先标定轮半径和地面/轮胎摩擦参数。

## 已知限制

- `wrist_cameras.yaml` 的实机标定字段仍是占位值；
- 尚未完成平墙深度误差和 `K + pose` 重投影误差测试；
- `capture_wrist_dataset.py` 的 `square` 运动模式是简单时序控制，不是闭环
  方形轨迹；
- 无头集成测试固定写入 `galbot_s1_headless_test` 目录，正式数据应使用
  时间戳录制；
- replay 的深度可视化 PNG 不能替代原始浮点深度。
