# 扩大版局部作业区：`scene_with_collision.usdz`

状态：**候选区域**，坐标尚待人工确认。此配置是显式选择的独立方案；原始 USDZ 和先前的小区域方案均保留。

## 区域与修补

- 统一配置：`/root/gpufree-data/galbot_ws/configs/scene_with_collision_expanded.yaml`。起点 `(9,0)`；多边形顶点及编号见 `candidate_preview.png`。区域沿主通道向东扩展至 `x=29`，向南包住西、中、东三处完整障碍，并绕开不完整的大型货架。机器人圆形 footprint 半径 `0.50 m`，另加 `0.20 m` 安全余量。当前未发现必须用可见几何修复的真实墙洞；不要把红色导航边界理解成实体墙。
- 工作场景：`/root/gpufree-data/data_usd/scene_local_expanded/local_work_scene.usd`；原场景基准：同目录 `local_source_only.usd`；独立可关闭修补层：同目录 `repair_layer.usd`。两块具名、不可见的碰撞地板位于扫描地面下方，顶面 `z=-0.24 m`，支撑主通道及东侧绕障区域。它们不参与相机渲染。关闭工作场景内 `/World/LocalRepairs` 的引用即可恢复原场景。重复运行生成脚本会重建同一两个 Prim，不叠加。
- 局部地图：同目录 `local_map.yaml` / `local_map.png`；导航配置：`navigation_local.yaml`；ROS profile：`/root/gpufree-data/sim_adapter/profiles/scene_with_collision_expanded.yaml`。原始 Occupancy Map 图像按区域裁剪并更新 origin，区域外设为 occupied，unknown 灰度另行保留，地图膨胀与底盘执行共用 footprint 约束。地图分辨率 `0.05 m`，尺寸 `581×311`，origin `(0.9749988556,-10.5250015259,0)`。碰撞网格在 `z[-0.085,1.415] m` 的投影额外占据 1280 格；没有对完整网格做碰撞分解。
- 生成报告 `build_report.json`。膨胀后可行驶空间为一个连通分量。`(23,-7)→(28,-7)` 直线穿过东侧障碍，不能通行；增加 `0.20 m` 路径规划余量后，可从障碍北侧绕行。实际机器人路线见 `validation/navigation/route_validation.json`。

## 深度问题和可见墙的取舍

腕部相机提供光轴方向 Z 深度 `distance_to_image_plane`，单位米。采集同时保存原始 `float32` 数组、有效 mask、原有兼容的毫米 PNG、RGB、相机内参和世界位姿。无穷大、NaN、零/负值、`0.10–40 m` 量程外值不参与反投影；没有把无回波写成最大距离。下游建图必须使用 mask 过滤。场景内的 Gaussian 外观与代理碰撞 Mesh 并不等价，RGB 中看见天花板不保证深度有回波。

在同一腕部相机起点和默认俯仰角下，将远裁剪从 `25 m` 增至 `40 m`，首帧左右无效比例从约 `4.26%/3.92%` 降至 `2.66%/2.45%`；增至 `80 m` 几乎没有进一步改善。扩大版选 `40 m`。固定视角中，西侧障碍无效 `0.398%`，中部障碍 `0.006%`，东侧障碍默认朝向 `6.998%`。东侧无效主要位于画面上部天花板/背景：上 160 行无效 `19.731%`，下 160 行 `0%`。同一位置的下俯角扫描约 `20°/22°/24°`，无效比例依次为 `1.915%/1.097%/0.554%`；`24°` 已切掉障碍上沿，故配置中的 `east_obstacle_aimed` 选择约 `22°`，兼顾完整障碍画面与低无效比例。这个固定检查视角不改变机器人腕部相机的全局默认角度。靠边视角无效 `7.308%`。此配置按每只相机 `3%` 无效比例阈值跳过质量差的帧，记录原因；优先在完整障碍附近采样。若下游只需要局部地面/障碍，可按任务选择感兴趣区域并继续使用 mask；所有**已保存帧**均保留原始全幅深度。

目前没有证据表明东侧上方无回波对应一个真实墙洞。在作业区外加可见墙会制造假的深度平面，还可能遮挡正常通道。这里仅用导航禁行边界阻止越界；待实地或 GUI 确认真正的墙面缺损和位置后，再按实际表面尺寸在独立修补层增加可渲染薄墙，并分别验证碰撞与深度。

物理路线对照发现底盘世界 Z 存在短时抬升：同一条北侧绕行路线在工作场景内为 `0.113–0.552 m`，关闭修补层后为 `-0.039–0.697 m`。扫描地面在这些位置约 `z=-0.20 m`，但抬升的确切物理原因尚未定位；不可将平面路线通过等同于相机高度稳定。采集器会把每帧实际相机高度与统一配置的 `0.63±0.10 m` 比较，超限时按配置记录并跳过该帧。东侧连续采集前仍应在 GUI 中检查机器人轮地接触和高度曲线。

## 机器人与双腕相机的采集位姿

机器人名义起点是世界坐标 `(9,0)`、偏航 `0°`，朝向 `+X`。它在仿真中运动，所以数据集没有一个固定的全程位姿；每帧的机器人位姿和相机完整 `world_from_camera_opencv` 矩阵写在各相机的 `manifest.jsonl` 中。扩大版三帧烟测的首帧实测为：机器人根节点 `(9.0148,0.0024,0.1174) m`、偏航约 `-0.107°`；左腕相机 `(9.4371,0.3819,0.4136) m`，离配置地面 `0.604 m`，光轴世界朝向约偏航 `-0.315°`、俯角 `9.27°`；右腕相机 `(9.4377,-0.3751,0.4136) m`，离地 `0.604 m`，约偏航 `-1.064°`、俯角 `11.04°`。腕相机的名义视线是偏航 `0°`、俯角 `10°`，视场角 `70°`；东侧障碍 `east_obstacle_aimed` 的约 `22°` 俯角是**独立固定验证相机**，并非腕相机默认值。

当前场景的自动数据集入口是 `scripts/capture_auto_rgbd.py`。它接收本目录的工作场景、地图和统一作业区配置，按局部地图规划、检查完整 footprint，并按深度与相机高度质量门槛过滤帧。旧的 `scripts/capture_wrist_dataset.py` 固定读取原 warehouse 组合场景，且没有局部地图约束和有效深度 mask；不能直接给它传 `--scene` 来切换到当前场景。

## 无头随机路线与连续 RGB-D 数据集

扩大版作业区轮廓见 `scene_local_expanded/candidate_preview.png`；仍需人工确认，配置状态保持 `candidate`。机器人圆形 footprint 半径 `0.50 m` 加 `0.20 m` 余量，规划地图按合计 `0.70 m` 膨胀。随机路线的每一段还会按同一多边形和完整 footprint 检查，执行器逐步检查中间路径；这防止地图栅格在凹角处误放行。地图只表达已扫描的二维障碍，不能保证避开所有未重建或悬空的物体。路线会记录在各回合的 `trajectory_plans.jsonl`，`--episodes N` 可生成 N 个独立随机路线回合；实际是否走完目标取决于回合时长和物理状态。

默认配置的 `skip` 模式只落盘达标帧；需要连续时间序列时用 `--quality-action mark`，每个采样时刻都保存左右 RGB、`depth_raw/*.npy`、`depth_valid_mask/*.png`、相机内参、`manifest.jsonl` 和兼容旧格式的 `pose.txt`。`manifest` 中的 `frame_quality_accepted` 与 `frame_quality_failures` 标记能否用于建图或训练，`quality_events.jsonl` 记录不合格原因；质量不合格帧仍保留原始深度和 mask，不能把无效深度当作真实表面。统一配置还要求整段 `usable_frames/(saved_frames+skipped_frames) ≥ 0.50`，低于此比例时报告为 `FAIL`，以免仅有少数可用帧的路线被当作合格数据集。`--min-target-distance` 控制目标至少离当前位置多远，`--duration` 控制每回合仿真时长；轨迹是否完整走完由报告的 `completed_targets` 判断。

目前保留的正式样例是 `--duration 40` 的 `captures/rgbd_extended_route_01/`，使用 `--quality-action mark`。报告为 `PASS`：机器人实际走 `14.10 m`，完成两个目标并开始第三条路线；左右各保存 `201` 张连续 RGB、原始深度、有效 mask 和相机位姿，`150` 组合格、`51` 组标记异常。后段转向后质量明显下降，因此不能把 201 组全算作有效几何观测。独立核验逐一检查了 `402` 张原始深度与 mask、`402` 个 `pose.txt` 矩阵与 manifest、一致的双相机时间戳、`200` 段实跑路径和 `3` 条规划路线，结果见 `dataset_audit.json`；轨迹见 `trajectory_preview.png`。合格帧的最大无效深度比例为左 `2.87%`、右 `2.76%`。实跑覆盖约 `x=3.52–9.01 m, y=-1.12–1.13 m`，主要是西侧通道往返；总路程 14.10 m 不等于扩大到东侧障碍区域。

高可用比例的连续前缀为 `frame_id=0–160`、仿真前 `32 s`：实走 `11.13 m`，`161` 组中 `147` 组合格（`91.3%`），见 `recommended_window.json`。这只是完整数据集的读取建议，所有帧仍按原编号保留。

后台新建一个空输出目录并显式检查报告状态；当前 Isaac Sim 的 `python.sh` 曾在报告为 `FAIL` 时仍返回 0，不能仅凭其进程退出码判断采集成功：

```bash
cd /root/gpufree-data/galbot_ws
GALBOT_CAPTURE_DIR=$(mktemp -d /root/gpufree-data/data_usd/scene_local_expanded/captures/rgbd_run_XXXXXXXX)
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/capture_auto_rgbd.py \
  --scene /root/gpufree-data/data_usd/scene_local_expanded/local_work_scene.usd \
  --map /root/gpufree-data/data_usd/scene_local_expanded/local_map.yaml \
  --workspace-config configs/scene_with_collision_expanded.yaml \
  --start 9 0 0 --motion-source internal --episodes 1 --duration 40 \
  --sample-rate 5 --seed 7 --min-target-distance 5 --speed 0.4 \
  --quality-action mark --output "$GALBOT_CAPTURE_DIR" \
  --ext-folder /root/gpufree-data/galbot_ws/exts \
  --enable galbot.mobility_gen --enable isaacsim.replicator.mobility_gen.examples
python3 scripts/check_capture_report.py "$GALBOT_CAPTURE_DIR/validation_report.json" --min-usable-fraction 0.50
```

东侧覆盖尚未达到同样质量：从 `(9,0,0°)` 朝 `(27.15,-8.70)` 的跨区路线运行到 `11.2 s` 时，57 组里仅 8 组合格，相机在约 `1.6 s` 后持续偏离，结果归档在 `diagnostics/east_route_camera_instability/`。中心 `(15,-1.5,0°)` 短探针 21 组均不合格；东侧障碍 `(23,-7,0°)` 到 `(28,-7)` 的短探针 12 组均不合格，分别归档在 `diagnostics/center_start_quality_failure/` 和 `diagnostics/east_obstacle_capture_failure/`。这两处在采集前的关节预热后，双臂关节误差分别已达约 `1.69 rad` 和 `1.56 rad`，而西侧可用起点约 `0.006 rad`。**已验证的是关节偏离目标；是否由扫描几何接触、地面或其他 PhysX 状态造成，仍待 GUI/接触信息确认。**统一配置增加 `max_initial_arm_error_rad: 0.25` 起步门槛；东侧重跑在写入第一帧前以 `right_arm_joint2` 误差 `-1.559 rad` 被拒绝，报告见 `diagnostics/east_preflight_gate/validation_report.json`。

`previews/capture_quality_routes.png` 将合格西侧路线与中、东侧失败探针标在同一张局部地图上。跨区路线的橙色部分虽能执行，仍需先解决相机姿态和手臂关节问题，才可把该区域计入有效采集范围。

固定向东目标 `(13,-1)` 的复现路线在约 `t=1.6 s`、机器人 `(9.66,-0.09)` 时出现底盘根节点高度由约 `0.13 m` 升到 `0.27 m`、左相机离地约 `0.94 m` 且俯仰误差约 `55°`。机器人最终到达目标，但 48 组中仅前 8 组合格。该次旧版报告只检查存在可用帧，曾给出结构状态 `PASS`；它已归档到 `diagnostics/east_edge_fixed_goal_failure/`，不能作为合格数据集。加入整段可用率门槛后，5 秒无头重跑为 `8/26=30.8%`，`validation_report.json` 正确给出 `FAIL`，见 `diagnostics/east_usable_gate_trial/`。源 USD 碰撞 Mesh 在异常位置 `2 m` 半径内的近地顶点高度仅约 `-0.205–-0.165 m`，未发现明显高出地面的障碍顶点，数值见同目录 `collision_floor_probe.json`。**已验证的是高度和相机姿态突变；哪一处接触或物理求解状态导致该现象仍未确认。**

另试过 `(9,0,135°)` 起点：机器人可规划且完成一个目标，但相机离地仅约 `0.17 m` 起步，31 次采样均因高度或朝向失稳而被拒绝，故该回合不提供有效采集帧；诊断在 `diagnostics/yaw135_physics_failure/`。当前不要把任意起始朝向都视为已验证可用。

无头连续采集已在该工作场景实测。初次 4 秒试跑虽通过深度和高度门槛，但末帧左相机光轴实测偏航 `-166.9°`、俯仰 `+60.0°`，与机器人前视目标明显不符。该次报告、manifest 和异常帧归档在 `diagnostics/pre_attitude_gate_trial/`。因此统一配置增加相机目标偏航 `0°`、俯仰 `-10°` 以及各 `10°` 容差，采集器按每帧实际相机光轴对机器人朝向检查。加门槛后同样的 4 秒单回合重跑通过验证，左右相机各保存 7 帧，跳过 14 个候选采样时刻；其中 9 次涉及相机朝向超限，7 次涉及深度无效比例超过 3%，7 次涉及相机高度超出 `0.63±0.10 m`，这些原因有重叠。`diagnostics/auto_dataset_attitude_checked/validation_report.json` 和 `episode_000/quality_events.jsonl` 分别记录整体验证及逐次跳帧原因。保存帧的左右相机偏航误差分别在 `-1.2°–2.7°`、`-1.6°–7.6°`，俯仰误差在 `-0.5°–3.5°`、`-2.8°–1.1°`。每帧均含 RGB、原始浮点深度、有效深度 mask、内参及相机/机器人位姿。

时长模式允许质量过滤后帧数少于采样次数。之前定额模式曾尝试 2 回合、每回合 10 帧，仅分别通过 8、6 帧，状态为 `FAIL`；摘要和跳帧记录在 `diagnostics/auto_dataset_quota_trials/`。当前可以无头采集经门槛筛选的数据，尚不能保证连续路线每回合的固定帧数。物理抬升、相机朝向漂移和深度无效的场景原因仍需定位，不应放宽门槛把坏帧算作合格数据。

## 重现命令

```bash
cd /root/gpufree-data/galbot_ws
python3 scripts/build_local_warehouse.py --workspace configs/scene_with_collision_expanded.yaml --output /root/gpufree-data/data_usd/scene_local_expanded
python3 /root/gpufree-data/sim_adapter/scripts/validate_scene_profile.py /root/gpufree-data/sim_adapter/profiles/scene_with_collision_expanded.yaml
python3 -m pytest tests/test_auto_navigation.py tests/test_local_workspace.py -q
```

同位姿原场景/修补层比较。各视角目录含 `rgb.png`、`depth_raw.npy`、`depth_valid_mask.png`、`points_world.npy`，全景汇总见 `validation/before_after.png`，东侧聚焦对照见 `validation/east_obstacle_aimed_before_after.png`：

```bash
cd /root/gpufree-data/galbot_ws
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/validate_local_views.py --scene /root/gpufree-data/data_usd/scene_local_expanded/local_source_only.usd --workspace-config configs/scene_with_collision_expanded.yaml --output /root/gpufree-data/data_usd/scene_local_expanded/validation/before
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/validate_local_views.py --scene /root/gpufree-data/data_usd/scene_local_expanded/local_work_scene.usd --workspace-config configs/scene_with_collision_expanded.yaml --output /root/gpufree-data/data_usd/scene_local_expanded/validation/after
python3 scripts/compare_local_views.py --before /root/gpufree-data/data_usd/scene_local_expanded/validation/before --after /root/gpufree-data/data_usd/scene_local_expanded/validation/after --output /root/gpufree-data/data_usd/scene_local_expanded/validation/before_after.png
python3 scripts/compare_local_views.py --before /root/gpufree-data/data_usd/scene_local_expanded/validation/before --after /root/gpufree-data/data_usd/scene_local_expanded/validation/after --view east_obstacle_aimed --include-mask --output /root/gpufree-data/data_usd/scene_local_expanded/validation/east_obstacle_aimed_before_after.png
```

底盘物理场景绕障：

```bash
cd /root/gpufree-data/galbot_ws
PYTHONUNBUFFERED=1 /root/isaacsim/python.sh scripts/validate_expanded_route.py --scene /root/gpufree-data/data_usd/scene_local_expanded/local_work_scene.usd --map /root/gpufree-data/data_usd/scene_local_expanded/local_map.yaml --workspace-config configs/scene_with_collision_expanded.yaml --start 23 -7 --goal 28 -7 --output /root/gpufree-data/data_usd/scene_local_expanded/validation/navigation/route_validation.json --ext-folder /root/gpufree-data/galbot_ws/exts --enable galbot.mobility_gen --enable isaacsim.replicator.mobility_gen.examples
```

GUI 启动与 ROS 导航（另开两个终端）：

```bash
/root/gpufree-data/sim_adapter/scripts/start_isaac_navigation.sh /root/gpufree-data/sim_adapter/profiles/scene_with_collision_expanded.yaml
```

```bash
source /opt/ros/humble/setup.bash
source /root/gpufree-data/navigation/install/setup.bash
ros2 launch galbot_navigation_bringup navigation.launch.py use_sim_time:=true params_file:=/root/gpufree-data/data_usd/scene_local_expanded/navigation_local.yaml
```

GUI 中选择 `GalbotS1Robot` 与 `ROS2NavigationScenario`，加载此 profile 的工作场景和 `local_map.yaml`。不要误用工作区根目录的 `MobilityGenData/map.yaml`。

无头 ROS `/cmd_vel` 验收可独立运行：

```bash
env -u PYTHONPATH -u PYTHONHOME GALBOT_SIM_PROFILE=/root/gpufree-data/sim_adapter/profiles/scene_with_collision_expanded.yaml GALBOT_NAV_TEST_STAGE=/root/gpufree-data/data_usd/scene_local_expanded/local_work_scene.usd GALBOT_ROBOT_PRIM_PATH=/World/robot ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH=/root/isaacsim/exts/isaacsim.ros2.bridge/bin:/root/isaacsim/exts/isaacsim.ros2.bridge/humble/lib PYTHONUNBUFFERED=1 /root/isaacsim/python.sh /root/gpufree-data/sim_adapter/scripts/test_ros2_adapter_physics.py
```

## 验证边界

八个固定视角的原始深度、有效 mask、反投影点云在开启/关闭不可见碰撞修补层后完全相同，RGB 仅有运行间渲染差异（各视角平均绝对差 `0.03–1.65/255`）；没有用修补层伪造可见表面。东侧障碍 A* 绕行实跑终点距目标 `0.219 m`，全部轨迹采样点保持在地图和区域约束内。三帧双相机实采均通过 `3%` 质量阈值和相机高度门槛，左相机无效 `2.639%–2.668%`，右相机 `2.403%–2.479%`；越界探针指令位移 `0.10 m`，实际 `0 m`，原因 `workspace_boundary`。当前 ROS 导航代码的 `AStarPlanner` 在此地图上可规划 `(23,-7)→(28,-7)`，拒绝区外目标 `(30,-7)`；扩大版 ROS adapter 物理验收中，前进 `0.400 m`、旋转 `0.600 rad`，越界指令位移 `0 m`。重复运行生成脚本后仍为两个修补 Prim，原 USDZ SHA-256 保持 `05c7696e24846569085b26fef0f71816d96a0db0a2852bb94319aa291b7884a8`。地图坐标、地图外禁行及配置读取的自动测试通过。实际 ROS GUI/RViz 目标规划仍待联机确认；区域边界和机器人现场起点需人工核对，确认前请保持 `candidate`，不要设为所有现有任务默认值。
