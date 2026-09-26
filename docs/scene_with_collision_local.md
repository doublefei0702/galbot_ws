# `scene_with_collision.usdz` 局部作业区（候选）

需要覆盖完整障碍物和绕障路线时，使用[扩大版候选区](scene_with_collision_expanded.md)。本页保留原来的小区域验收记录。

## 检查结论

- 目标文件是 `/root/gpufree-data/data_usd/scene_with_collision.usdz`，Isaac Sim 5.0.0 环境；USD 单位为米，Z 向上，默认 Prim 为 `/World`。NuRec Gaussian Volume 的包围盒约为 `x[-27.52,89.77], y[-121.11,15.60], z[-3.39,10.82]`。它的代理是单个普通 Mesh（734,459 顶点、1,056,369 三角面），该 Mesh 带 `CollisionAPI` 和 `MeshCollisionAPI`。未修改、未对其做碰撞分解。
- 原始 `map_0_1.yaml` 来自 Isaac Occupancy Map 工具；它的图片有 6,115,084 个白色自由格，其中 2,338,479 个格子位于碰撞 Mesh 的轴对齐包围盒之外，说明不能直接把白色当作整个仓库都可行驶。YAML 不包含当时的 Z 采样设置，因此无法从文件恢复工具的高度参数。
- 地图和三角网格共同支持一个保守候选区：`[(7,-2),(12,-2),(12,2),(7,2)]`，起点 `(9,0)`。区域内水平、近地面三角面面积约 20.4 m²，地面中位高度约 `-0.183 m`；所选机器人高度带 `[-0.085,1.415] m` 内没有碰撞三角面投影到该候选区。此处没有定位到应补的真实墙面或可见地面缺口。首次机器人运行时底盘下陷，因此修补层加入一个局部、不可见、仅供碰撞的地板支撑，顶面 `-0.215 m`，低于测得的扫描地面最低值约 `0.011 m`。配置状态仍是 `candidate`，人工确认前不作为其他任务默认值。
- 已确认 `GalbotS1Robot.write_action()` 直接积分速度并更新世界位姿；单靠 PhysX 碰撞网格挡不住底盘。启用候选配置时，执行前会检查完整圆形 footprint（半径 `0.50 m` 加安全余量 `0.20 m`）和运动中间路径。ROS 场景的 `/cmd_vel` 也调用这条检查，并在阻挡时丢弃当前命令、输出原因。
- MobilityGen 原版 `enable_depth_rendering()` 绑定 `distance_to_camera`（径向距离），与原采集协议声称的光轴 Z 深度不符。腕部新采集改为 `distance_to_image_plane`；旧数据不变，不能按 Z 深度反投影。新采集保留原始 float32 深度，并额外保存有效 mask；NaN、无穷大、零/负值和配置量程外像素均无效。相机内参与位姿协议维持原有格式。
- 原导航臂姿的目标关节值虽已配置，但仅靠物理步等待，手臂在该扫描场景内不能稳定到达目标，曾导致左相机大面积自遮挡。显式启用本候选配置时，采集脚本和 ROS 场景会在物理步开始前把手臂关节设到**已有的导航姿态**，随后继续由现有控制保持；不改变相机内外参。局部配置的实际腕部相机标称高度为扫描地面上方 `0.63 m`，允许 `0.10 m` 偏差。下文四个静态检查视角用单独固定世界坐标，其中相机高度为 `1.20 m`，用于排查表面，不代表机器人腕部安装高度。

## 产物与坐标

- 统一配置：`/root/gpufree-data/galbot_ws/configs/scene_with_collision_local.yaml`
- 生成目录：`/root/gpufree-data/data_usd/scene_local_candidate/`
  - `local_work_scene.usd`：引用原 USDZ 和 `repair_layer.usd`；`/World/LocalRepairs` 可关闭。
  - `local_source_only.usd`：相同原场景引用，没有修补层，用于同位姿基准。
  - `repair_layer.usd`：独立修补几何文件，当前 1 个 `LocalCollisionFloorSupport` 碰撞 Prim；设为 invisible，不参与 RGB-D 渲染。可从工作场景中禁用 `/World/LocalRepairs` 的引用以恢复原场景。
  - `local_map.yaml` / `local_map.png`：原图按候选区域裁剪并更新 origin；区外写为 occupied，原 unknown 灰度仍单独保留。近地面碰撞 Mesh 按配置高度带复核并合并到地图。地图用于真实规划，而 `candidate_preview.png` 只供查看。
  - `navigation_local.yaml`：从现有导航配置生成，仅把 `footprint_radius` 调到 `0.70 m`。
  - `validation/before_after.png`：四个固定相机位姿的原场景与修补层对比；`validation/arm_initialization_comparison.png`：相同机器人起点下，左右腕相机在关节初始化前后的实采画面对比。后者的相机位姿随手臂改变，不用于评估几何修补。
- ROS profile：`/root/gpufree-data/sim_adapter/profiles/scene_with_collision_local.yaml`，指向上述工作场景、地图和统一配置。
- 局部地图 origin 为 `(5.9749988556,-3.0250015259,0)`，分辨率 `0.05 m`，图像 `141×121`；世界点 `(9,0)` 对应栅格 `(60,60)`。生成脚本同时处理地图 origin 旋转及图像纵轴翻转。

## 运行命令

```bash
cd /root/gpufree-data/galbot_ws
python3 scripts/build_local_warehouse.py --workspace configs/scene_with_collision_local.yaml
python3 /root/gpufree-data/sim_adapter/scripts/validate_scene_profile.py /root/gpufree-data/sim_adapter/profiles/scene_with_collision_local.yaml
python3 -m pytest tests/test_auto_navigation.py tests/test_local_workspace.py -q
```

固定视角 RGB、原始 Z 深度、有效 mask 和按真实 K/位姿反投影的点云：

```bash
cd /root/gpufree-data/galbot_ws
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/validate_local_views.py --scene /root/gpufree-data/data_usd/scene_local_candidate/local_source_only.usd --workspace-config configs/scene_with_collision_local.yaml --output /root/gpufree-data/data_usd/scene_local_candidate/validation/before
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/validate_local_views.py --scene /root/gpufree-data/data_usd/scene_local_candidate/local_work_scene.usd --workspace-config configs/scene_with_collision_local.yaml --output /root/gpufree-data/data_usd/scene_local_candidate/validation/after
python3 scripts/compare_local_views.py --before /root/gpufree-data/data_usd/scene_local_candidate/validation/before --after /root/gpufree-data/data_usd/scene_local_candidate/validation/after --output /root/gpufree-data/data_usd/scene_local_candidate/validation/before_after.png
```

无头机器人采集与边界探针（单独目录，保留现有输出）：

```bash
cd /root/gpufree-data/galbot_ws
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/capture_auto_rgbd.py --scene /root/gpufree-data/data_usd/scene_local_candidate/local_work_scene.usd --map /root/gpufree-data/data_usd/scene_local_candidate/local_map.yaml --workspace-config configs/scene_with_collision_local.yaml --start 9 0 0 --output /root/gpufree-data/data_usd/scene_local_candidate/captures/capture_smoke --smoke --frames 3 --boundary-probe --ext-folder /root/gpufree-data/galbot_ws/exts --enable galbot.mobility_gen --enable isaacsim.replicator.mobility_gen.examples
```

GUI 与 ROS 导航使用两个终端。GUI 中打开工作场景，MobilityGen 的 Occupancy Map 填 `local_map.yaml`，选择 `GalbotS1Robot` 和 `ROS2NavigationScenario`，按 Build、Play。工作地图位于 `scene_local_candidate`，不要误用 `MobilityGenData/map.yaml`：

```bash
/root/gpufree-data/sim_adapter/scripts/start_isaac_navigation.sh /root/gpufree-data/sim_adapter/profiles/scene_with_collision_local.yaml
```

```bash
source /opt/ros/humble/setup.bash
source /root/gpufree-data/navigation/install/setup.bash
ros2 launch galbot_navigation_bringup navigation.launch.py use_sim_time:=true params_file:=/root/gpufree-data/data_usd/scene_local_candidate/navigation_local.yaml
```

无头 ROS `/cmd_vel` 物理步验收（使用相同 profile 和工作场景）：

```bash
env -u PYTHONPATH -u PYTHONHOME GALBOT_SIM_PROFILE=/root/gpufree-data/sim_adapter/profiles/scene_with_collision_local.yaml GALBOT_NAV_TEST_STAGE=/root/gpufree-data/data_usd/scene_local_candidate/local_work_scene.usd GALBOT_ROBOT_PRIM_PATH=/World/robot ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH=/root/isaacsim/exts/isaacsim.ros2.bridge/bin:/root/isaacsim/exts/isaacsim.ros2.bridge/humble/lib PYTHONUNBUFFERED=1 /root/isaacsim/python.sh /root/gpufree-data/sim_adapter/scripts/test_ros2_adapter_physics.py
```

## 验证记录与待确认项

- 四个同位姿视角（设备表面、地面、正常通道、作业区边缘）的 Z 深度有效率分别是 `98.824%`、`99.940%`、`95.869%`、`95.641%`。启用碰撞地板后，四组原始深度、有效 mask、反投影点云完全一致，RGB 平均绝对像素差约 `0.03–0.10/255`（Gaussian 渲染的运行间微小差异），没有新增遮挡或有效深度。无效深度零星落在设备细节和远处，不能仅凭这些值认定存在墙洞。
- 初次三帧机器人实采保存了左右 RGB、原始深度、mask、相机内参与世界位姿。底盘世界 Z 从无支撑时约 `-0.05～-0.07 m` 提高到有支撑时 `0.14～0.15 m`。初次采集时左相机上方约四分之一被机器人自身遮挡。单纯延长关节稳定时间、或在测试场景关闭原扫描网格碰撞，都没有消除该问题；正式工作场景仍保留原扫描网格碰撞。将已有导航臂姿在启动时设为关节初值后，`captures/capture_ready` 的最大手臂关节误差约 `0.013 rad`，左右相机高度均为局部地面上方约 `0.62～0.64 m`，明显遮挡消失，仅画面上角仍有少量机器人边缘。三帧的深度无效率约 `4%`；深度有效率不能代表自遮挡程度，原始图像应由数据消费者按任务需求复核。
- `captures/capture_ready` 的越界探针从 `(11.25,0)` 指令移动 `0.10 m`，实际移动 `0 m`，原因 `workspace_boundary`。采集脚本保存了关节目标误差和每帧实际相机高度；同一局部配置也控制 ROS 场景的关节初值。
- 初次右相机首帧原始 `float32` 深度有 11,585 个 `+Inf` 无回波像素；有效 mask 把它们剔除，保留原始数组，未填成 `25 m`。同一工作场景的无头 ROS `/cmd_vel` 验收通过：向前命令 400 个物理步得到 `0.40000 m` 位移，旋转命令 400 个物理步得到 `0.60000 rad`；从 `(11.25,0)` 向区外发布速度命令时，日志记录 `workspace_boundary`，位移 `0.000000 m`。
- 地图测试：`(9,0)→(10.5,0)` 和 `(9,0)→(10.5,1)` 在膨胀地图内可规划；`(13,0)` 位于区外不可规划；靠边的 `(11.6,0)` 中心虽在地图白格，完整 footprint 不可用。
- 候选坐标与真实机器人起点仍须人工确认。ROS GUI、RViz 目标规划与现场完整任务还未做联机验收。下游建图代码不在本仓库；消费者必须以 `depth_valid_mask` 或 `isfinite(depth) & (0.1 <= depth <= 25)` 过滤，再用 Z 深度和 `world_from_camera_opencv` 反投影，不能把无穷远填成量程上限。逐帧清单记录实际相机高度及是否低于配置容差下限。
