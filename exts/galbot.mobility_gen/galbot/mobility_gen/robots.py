# SPDX-License-Identifier: Apache-2.0
"""GalbotS1Robot: MobilityGen 机器人实现。

关键事实(全部实测验证, 详见 galbot_ws/README.md):
- 官方 USD 是固定基座(root_joint 焊死世界)且包装层无法干净修复 → 必须用
  stages/galbot_s1_sim_src 派生副本(make_sim_variant.py 生成, drive 修正已烘焙)。
- 默认变体 Robot=none 只有骨架, 必须显式选 Robot=robot/Sensor=sensors/Physics=physx。
- 关节按名称解析索引, 不依赖 USD 排列顺序。
- action = [v, wz]; SwerveController 输出 (转向 position target, 驱动 velocity target)。
"""
from __future__ import annotations

import math
import os

import numpy as np
import omni.usd
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.replicator.mobility_gen.impl.common import Buffer
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS, MobilityGenRobot
from isaacsim.replicator.mobility_gen.impl.types import Pose2d
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world

from .swerve_controller import SwerveController
from .pose_utils import aimed_world_camera_quat

_GALBOT_WS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
GALBOT_USD = os.environ.get(
    "GALBOT_S1_USD",
    os.path.join(_GALBOT_WS, "stages", "galbot_s1_sim_src", "galbot_s1.usda"),
)

STEERING_JOINTS = ["wheel_steering_joint1", "wheel_steering_joint2",
                   "wheel_steering_joint3", "wheel_steering_joint4"]
DRIVING_JOINTS = ["wheel_driving_joint1", "wheel_driving_joint2",
                  "wheel_driving_joint3", "wheel_driving_joint4"]
# 采集臂姿从 configs/arm_pose.yaml 加载(default_pose 指定的姿态, 未列出的关节=0)。
# 可用 scripts/preview_arm_pose.py 标定新姿态(基于 USD 运动学解算+仿真验证)。
import yaml as _yaml

_ARM_POSE_PATH = os.path.join(_GALBOT_WS, "configs", "arm_pose.yaml")


def _load_arm_pose() -> dict:
    try:
        cfg = _yaml.safe_load(open(_ARM_POSE_PATH))
        pose = cfg.get(cfg.get("default_pose", "nav"), {})
        return {k: float(v) for k, v in pose.items()}
    except Exception:
        return {}


def _load_wrist_view() -> tuple[float, float]:
    try:
        cfg = _yaml.safe_load(open(os.path.join(_GALBOT_WS, "configs", "wrist_cameras.yaml")))
        view = cfg.get("navigation_view", {})
        return float(view.get("pitch_deg", -10.0)), float(view.get("yaw_deg", 0.0))
    except Exception:
        return -10.0, 0.0


ARM_HOLD_POSE = {name: 0.0 for name in (
    [f"left_arm_joint{i}" for i in range(1, 8)]
    + [f"right_arm_joint{i}" for i in range(1, 8)]
    + ["torso_lift_joint1", "head_joint1", "head_joint2",
       "left_active_joint1", "left_active_joint2", "left_passive_joint",
       "right_active_joint1", "right_active_joint2", "right_passive_joint"]
)}
ARM_HOLD_POSE.update(_load_arm_pose())
WRIST_VIEW_PITCH_DEG, WRIST_VIEW_YAW_DEG = _load_wrist_view()



@ROBOTS.register()
class GalbotS1Robot(MobilityGenRobot):
    # --- 基础 ---
    physics_dt: float = 1.0 / 200.0
    z_offset: float = 0.05

    # --- 追踪相机 ---
    chase_camera_base_path = "base_link"
    chase_camera_x_offset: float = -1.5
    chase_camera_z_offset: float = 1.2
    chase_camera_tilt_angle: float = 55.0

    # --- 占据图采样(底盘 0.5×0.5m, 轮距 ±0.247) ---
    occupancy_map_radius: float = 0.5
    occupancy_map_z_min: float = 0.05
    occupancy_map_z_max: float = 0.5
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.45

    # --- 遥控增益 ---
    keyboard_linear_velocity_gain: float = 0.5
    keyboard_angular_velocity_gain: float = 1.0
    gamepad_linear_velocity_gain: float = 0.5
    gamepad_angular_velocity_gain: float = 1.0

    # --- 随机动作/路径跟踪 ---
    random_action_linear_velocity_range = (-0.3, 0.3)
    random_action_angular_velocity_range = (-0.8, 0.8)
    random_action_linear_acceleration_std: float = 0.8
    random_action_angular_acceleration_std: float = 2.0
    random_action_grid_pose_sampler_grid_size: float = 5.0

    path_following_speed: float = 0.3
    path_following_angular_gain: float = 1.5
    path_following_stop_distance_threshold: float = 0.5
    path_following_forward_angle_threshold = math.pi / 4
    path_following_target_point_offset_meters: float = 1.0

    def __init__(self, prim_path: str, articulation_view: Articulation):
        self.prim_path = prim_path
        self.articulation_view = articulation_view
        self.controller = SwerveController(wheel_radius=0.08)
        self.action = Buffer(np.zeros(2))  # [v, wz]
        self.position = Buffer()
        self.orientation = Buffer()
        self.joint_positions = Buffer()
        self.joint_velocities = Buffer()
        self.linear_velocity = Buffer()
        self.angular_velocity = Buffer()
        self.left_wrist_camera = None
        self.right_wrist_camera = None
        self._idx_steer = None
        self._idx_drive = None
        self._idx_hold = None
        self._wrist_pose_handles = {}
        self._wrist_aim_steps = 0
        self._wrist_aimed = False

    # ---------------- 生命周期 ----------------

    @classmethod
    def build(cls, prim_path: str) -> "GalbotS1Robot":
        world = get_world()
        add_reference_to_stage(usd_path=GALBOT_USD, prim_path=prim_path)
        stage = omni.usd.get_context().get_stage()
        vss = stage.GetPrimAtPath(prim_path).GetVariantSets()
        vss.SetSelection("Robot", "robot")
        vss.SetSelection("Sensor", "sensors")
        vss.SetSelection("Physics", "physx")
        view = Articulation(prim_path)
        world.scene.add(view)
        instance = cls(prim_path=prim_path, articulation_view=view)
        instance.build_wrist_cameras()
        return instance

    # 腕部 RGB-D 相机: 挂在 *_arm_camera_link 下(随臂运动)。
    # Camera prim 仍挂在真机腕部 link 下。具体本地旋转不硬编码：双臂稳定后按
    # base 坐标将两路精确对准前下方，避开左右 link 镜像轴与 Camera 轴约定陷阱。
    WRIST_CAMERA_SPECS = {
        "left": "left_arm_link7/left_arm_camera_link/sensors/rgbd_camera",
        "right": "right_arm_link7/right_arm_camera_link/sensors/rgbd_camera",
    }
    wrist_camera_resolution = (640, 480)

    def build_wrist_cameras(self):
        """在左右腕部 camera_link 下创建 Camera prim 并挂为 MobilityGen 模块。

        挂载在刚体 link 下 → 相机世界位姿跟随臂关节运动;
        MobilityGenCamera 的 rgb/depth buffer 进模块树 → replay 自动渲染腕部数据。
        """
        from isaacsim.replicator.mobility_gen.impl.camera import MobilityGenCamera
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        from isaacsim.sensors.camera import Camera

        for side, rel_path in self.WRIST_CAMERA_SPECS.items():
            path = f"{self.prim_path}/{rel_path}"
            UsdGeom.Camera.Define(stage, path)
            self._wrist_pose_handles[side] = Camera(prim_path=path)
            mg_cam = MobilityGenCamera(path, self.wrist_camera_resolution)
            setattr(self, f"{side}_wrist_camera", mg_cam)

    def _aim_wrist_cameras_once(self):
        """Author fixed local rotations after the commanded arm pose has settled."""
        if self._wrist_aimed:
            return
        self._wrist_aim_steps += 1
        if self._wrist_aim_steps < 400:  # 2 s at 200 Hz; wait for both arms to converge
            return
        _, base_q = self.articulation_view.get_world_poses()
        aim_q = aimed_world_camera_quat(
            np.asarray(base_q).reshape(-1, 4)[0], WRIST_VIEW_PITCH_DEG, WRIST_VIEW_YAW_DEG)
        for camera in self._wrist_pose_handles.values():
            camera.set_world_pose(orientation=aim_q, camera_axes="world")
        self._wrist_aimed = True

    def initialize(self) -> bool:
        """physics ready 后按名称解析关节索引(不依赖 USD 顺序)。

        可重入: dof_names 未就绪(物理未初始化)时返回 False, 下一物理步重试。
        UI 流程不会主动调用此方法 → write_action 懒初始化。
        """
        dof_names = self.articulation_view.dof_names
        if not dof_names:
            return False
        names = list(dof_names)
        self._idx_steer = np.array([names.index(n) for n in STEERING_JOINTS])
        self._idx_drive = np.array([names.index(n) for n in DRIVING_JOINTS])
        hold = [n for n in ARM_HOLD_POSE if n in names]
        self._idx_hold = np.array([names.index(n) for n in hold])
        self._hold_targets = np.array([ARM_HOLD_POSE[n] for n in hold], dtype=np.float32)
        return True

    # ---------------- 控制与状态(MobilityGen 协议) ----------------

    def write_action(self, step_size: float):
        if self._idx_steer is None:
            if not self.initialize():
                return  # 物理视图未就绪, 本步跳过(下一步重试)
        v, wz = self.action.get_value()
        cmd = np.array([float(v), 0.0, float(wz)])
        pos = np.asarray(self.articulation_view.get_joint_positions()).flatten()
        targets = self.controller.forward_with_state(cmd, list(pos[self._idx_steer]))
        steer_t = np.array([t[0] for t in targets], dtype=np.float32)
        drive_w = np.array([t[1] for t in targets], dtype=np.float32)
        self.articulation_view.set_joint_position_targets(self._hold_targets, joint_indices=self._idx_hold)
        self.articulation_view.set_joint_position_targets(steer_t, joint_indices=self._idx_steer)
        self.articulation_view.set_joint_velocity_targets(drive_w, joint_indices=self._idx_drive)
        self._aim_wrist_cameras_once()

    def update_state(self):
        p, q = self.articulation_view.get_world_poses()
        self.position.set_value(np.asarray(p).flatten()[:3])
        self.orientation.set_value(np.asarray(q).flatten()[:4])
        self.joint_positions.set_value(np.asarray(self.articulation_view.get_joint_positions()).flatten())
        self.joint_velocities.set_value(np.asarray(self.articulation_view.get_joint_velocities()).flatten())
        self.linear_velocity.set_value(np.zeros(3))
        self.angular_velocity.set_value(np.zeros(3))
        # 关键: 传播到子模块(前相机等), 否则 rgb buffer 永不更新
        for child in self.children().values():
            child.update_state()

    def write_replay_data(self):
        self.articulation_view.set_world_poses(
            positions=np.asarray(self.position.get_value())[None, :])
        self.articulation_view.set_joint_positions(
            np.asarray(self.joint_positions.get_value())[None, :])

    def set_pose_2d(self, pose: Pose2d):
        self.articulation_view.set_velocities(np.zeros((1, 6)))
        pos = np.zeros(3)
        pos[0], pos[1], pos[2] = pose.x, pose.y, self.z_offset
        quat = np.asarray(euler_angles_to_quat(np.array([0.0, 0.0, pose.theta]))).flatten()
        self.articulation_view.set_world_poses(positions=pos[None, :], orientations=quat[None, :])

    def get_pose_2d(self) -> Pose2d:
        pos, quat = self.articulation_view.get_world_poses()
        pos = np.asarray(pos).flatten()
        theta = quat_to_euler_angles(np.asarray(quat).flatten())[2]
        return Pose2d(x=float(pos[0]), y=float(pos[1]), theta=float(theta))
