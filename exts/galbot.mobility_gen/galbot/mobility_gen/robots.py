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
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.replicator.mobility_gen.impl.common import Buffer
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS, MobilityGenRobot
from isaacsim.replicator.mobility_gen.impl.types import Pose2d
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world

from .base_controller import PlanarBaseController
from .pose_utils import aimed_world_camera_quat, set_camera_horizontal_fov

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


def _load_wrist_view() -> tuple[float, float, float]:
    try:
        cfg = _yaml.safe_load(open(os.path.join(_GALBOT_WS, "configs", "wrist_cameras.yaml")))
        view = cfg.get("navigation_view", {})
        return (float(view.get("pitch_deg", -10.0)),
                float(view.get("yaw_deg", 0.0)),
                float(view.get("horizontal_fov_deg", 70.0)))
    except Exception:
        return -10.0, 0.0, 70.0


ARM_HOLD_POSE = {name: 0.0 for name in (
    [f"left_arm_joint{i}" for i in range(1, 8)]
    + [f"right_arm_joint{i}" for i in range(1, 8)]
    + ["torso_lift_joint1", "head_joint1", "head_joint2",
       "left_active_joint1", "left_active_joint2", "left_passive_joint",
       "right_active_joint1", "right_active_joint2", "right_passive_joint"]
)}
ARM_HOLD_POSE.update(_load_arm_pose())
WRIST_VIEW_PITCH_DEG, WRIST_VIEW_YAW_DEG, WRIST_HORIZONTAL_FOV_DEG = _load_wrist_view()

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

    def build_chase_camera(self) -> str:
        """Use Kit's Perspective camera and move it as a safe chase view.

        The original MobilityGen chase camera is parented below the dynamic
        articulation.  Its local-axis convention is wrong for this USD and
        can point the viewport into the robot, yielding a black screen.  The
        persistent Perspective camera has a known-good renderer setup; its
        world pose is updated from ``update_state`` instead.
        """
        return "/OmniverseKit_Persp"

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
        self.controller = PlanarBaseController()
        self.action = Buffer(np.zeros(3))  # [vx, vy, wz], body-frame Twist
        self.position = Buffer()
        self.orientation = Buffer()
        self.joint_positions = Buffer()
        self.joint_velocities = Buffer()
        self.linear_velocity = Buffer()
        self.angular_velocity = Buffer()
        self.left_wrist_camera = None
        self.right_wrist_camera = None
        self._idx_hold = None
        self._wrist_pose_handles = {}
        self._wrist_aim_steps = 0
        self._wrist_aimed = False
        self.wrist_view_pitch_deg = WRIST_VIEW_PITCH_DEG
        self.wrist_view_yaw_deg = WRIST_VIEW_YAW_DEG
        self._follow_viewport = os.environ.get("GALBOT_FOLLOW_VIEWPORT", "1").lower() in {
            "1", "true", "yes"}
        self._follow_viewport_steps = 0
        self._command_pose = None
        # Optional local-area gate, installed by the opt-in collector or ROS profile.
        self.motion_guard = None
        self.last_motion_block_reason = None

    # ---------------- 生命周期 ----------------

    @classmethod
    def build(cls, prim_path: str) -> "GalbotS1Robot":
        # MobilityGen supplies /World/robot by default.  The isolated ROS2
        # adapter can override it before Build so a customer profile remains
        # explicit about the robot prim without modifying Isaac's extension.
        configured_prim_path = os.environ.get("GALBOT_ROBOT_PRIM_PATH")
        if configured_prim_path:
            if not configured_prim_path.startswith("/"):
                raise ValueError("GALBOT_ROBOT_PRIM_PATH must be an absolute USD prim path")
            prim_path = configured_prim_path
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
        from isaacsim.replicator.mobility_gen.impl.utils.prim_utils import prim_get_world_transform
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        from isaacsim.sensors.camera import Camera

        for side, rel_path in self.WRIST_CAMERA_SPECS.items():
            path = f"{self.prim_path}/{rel_path}"
            UsdGeom.Camera.Define(stage, path)
            camera = Camera(prim_path=path, resolution=self.wrist_camera_resolution)
            set_camera_horizontal_fov(camera, WRIST_HORIZONTAL_FOV_DEG)
            self._wrist_pose_handles[side] = camera

            class _WristCamera(MobilityGenCamera):
                """MobilityGen camera that tolerates annotators before frame one."""

                def enable_depth_rendering(self):
                    # MobilityGen's default is distance_to_camera (radial range).
                    # The RGB-D protocol and pinhole backprojection require Z depth.
                    if self._render_product is None:
                        self.enable_rendering()
                    if self._depth_annotator is None:
                        import omni.replicator.core as rep
                        self._depth_annotator = rep.AnnotatorRegistry.get_annotator(
                            "distance_to_image_plane")
                        self._depth_annotator.attach(self._render_product)

                def update_state(self):
                    if self._rgb_annotator is not None:
                        data = np.asarray(self._rgb_annotator.get_data())
                        if data.ndim == 3 and data.size:
                            self.rgb_image.set_value(data[:, :, :3])
                    if self._depth_annotator is not None:
                        data = np.asarray(self._depth_annotator.get_data())
                        if data.ndim >= 2 and data.size:
                            self.depth_image.set_value(data)
                    position, orientation = prim_get_world_transform(self._prim)
                    self.position.set_value(position)
                    self.orientation.set_value(orientation)

            mg_cam = _WristCamera(path, self.wrist_camera_resolution)
            mg_cam.enable_rgb_rendering()
            mg_cam.enable_depth_rendering()
            setattr(self, f"{side}_wrist_camera", mg_cam)

    def camera_health(self) -> dict[str, bool]:
        """Report whether the follow viewport and both wrist sensors exist."""
        stage = omni.usd.get_context().get_stage()
        result = {"follow": stage.GetPrimAtPath("/OmniverseKit_Persp").IsValid()}
        for side, rel_path in self.WRIST_CAMERA_SPECS.items():
            path = f"{self.prim_path}/{rel_path}"
            result[side] = (
                stage.GetPrimAtPath(path).IsValid()
                and getattr(self, f"{side}_wrist_camera") is not None
            )
        return result

    def _aim_wrist_cameras_once(self):
        """Author fixed local rotations after the commanded arm pose has settled."""
        if self._wrist_aimed:
            return
        self._wrist_aim_steps += 1
        if self._wrist_aim_steps < 400:  # 2 s at 200 Hz; wait for both arms to converge
            return
        _, base_q = self.articulation_view.get_world_poses()
        aim_q = aimed_world_camera_quat(
            np.asarray(base_q).reshape(-1, 4)[0], self.wrist_view_pitch_deg, self.wrist_view_yaw_deg)
        for camera in self._wrist_pose_handles.values():
            camera.set_world_pose(orientation=aim_q, camera_axes="world")
        self._wrist_aimed = True

    def initialize(self) -> bool:
        """physics ready 后按名称解析需要保持的上身关节索引。

        可重入: dof_names 未就绪(物理未初始化)时返回 False, 下一物理步重试。
        UI 流程不会主动调用此方法 → write_action 懒初始化。
        """
        dof_names = self.articulation_view.dof_names
        if not dof_names:
            return False
        names = list(dof_names)
        hold = [n for n in ARM_HOLD_POSE if n in names]
        self._idx_hold = np.array([names.index(n) for n in hold])
        self._hold_targets = np.array([ARM_HOLD_POSE[n] for n in hold], dtype=np.float32)
        return True

    # ---------------- 控制与状态(MobilityGen 协议) ----------------

    def write_action(self, step_size: float):
        if self._idx_hold is None:
            if not self.initialize():
                return  # 物理视图未就绪, 本步跳过(下一步重试)
        if self._command_pose is None:
            self._command_pose = self._physical_pose_2d()
        pose = self._command_pose
        command = np.asarray(self.action.get_value(), dtype=float)
        self.last_motion_block_reason = None
        if self.motion_guard is not None:
            reason = self.motion_guard(pose, command, step_size)
            if reason is not None:
                self.last_motion_block_reason = reason
                self.action.set_value(np.zeros(3, dtype=float))
                self.articulation_view.set_velocities(np.zeros((1, 6), dtype=np.float32))
                self.articulation_view.set_joint_position_targets(self._hold_targets, joint_indices=self._idx_hold)
                return
        x, y, yaw = self.controller.integrate(
            pose.x, pose.y, pose.theta, command, step_size)
        self._command_pose = Pose2d(x=x, y=y, theta=yaw)
        p, q = self.articulation_view.get_world_poses()
        position = np.asarray(p).reshape(-1, 3)[0].copy()
        position[0], position[1] = x, y
        orientation = np.asarray(
            euler_angles_to_quat(np.array([0.0, 0.0, yaw]))).flatten()
        self.articulation_view.set_world_poses(
            positions=position[None, :], orientations=orientation[None, :])
        self.articulation_view.set_velocities(np.zeros((1, 6), dtype=np.float32))
        self.articulation_view.set_joint_position_targets(self._hold_targets, joint_indices=self._idx_hold)
        self._aim_wrist_cameras_once()

    def update_state(self):
        p, q = self.articulation_view.get_world_poses()
        self.position.set_value(np.asarray(p).flatten()[:3])
        self.orientation.set_value(np.asarray(q).flatten()[:4])
        self.joint_positions.set_value(np.asarray(self.articulation_view.get_joint_positions()).flatten())
        self.joint_velocities.set_value(np.asarray(self.articulation_view.get_joint_velocities()).flatten())
        # Articulation velocity is physical truth (world-frame linear and
        # angular xyz), unlike the former placeholder zero buffers.  Keep the
        # fallback for the small window before PhysX initializes the view.
        try:
            velocity = np.asarray(self.articulation_view.get_velocities()).reshape(-1, 6)[0]
            self.linear_velocity.set_value(velocity[:3].copy())
            self.angular_velocity.set_value(velocity[3:].copy())
        except Exception:
            self.linear_velocity.set_value(np.zeros(3))
            self.angular_velocity.set_value(np.zeros(3))
        self._update_follow_viewport(np.asarray(self.position.get_value()), np.asarray(self.orientation.get_value()))
        # 关键: 传播到子模块(前相机等), 否则 rgb buffer 永不更新
        for child in self.children().values():
            child.update_state()

    def _update_follow_viewport(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Keep the GUI camera behind and above the moving base.

        This is deliberately separate from both wrist RGB-D cameras: changing
        the operator viewport must never remove or reparent sensor cameras.
        """
        if not self._follow_viewport or position.size < 3 or orientation.size < 4:
            return
        # The simulation ticks at 200 Hz; a 20 Hz operator camera is smooth
        # while avoiding an unnecessary viewport update on every physics tick.
        self._follow_viewport_steps += 1
        if self._follow_viewport_steps % 10:
            return
        try:
            yaw = float(quat_to_euler_angles(orientation.flatten()[:4])[2])
            forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            base = position.flatten()[:3]
            eye = base - 2.4 * forward + np.array([0.0, 0.0, 1.5])
            target = base + 0.45 * forward + np.array([0.0, 0.0, 0.35])
            set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
        except Exception:
            # The viewport may not exist during headless construction or
            # shutdown.  Simulation, ROS and wrist cameras remain unaffected.
            return

    def write_replay_data(self):
        orientation = np.asarray(self.orientation.get_value())
        self.articulation_view.set_world_poses(
            positions=np.asarray(self.position.get_value())[None, :],
            orientations=orientation[None, :])
        self.articulation_view.set_joint_positions(
            np.asarray(self.joint_positions.get_value())[None, :])
        # Camera 的动态定向不属于关节状态，必须在 replay 中显式恢复；否则图像
        # 使用默认 USD Camera 朝向，而 common state 中仍是录制时的相机位姿。
        aim_q = aimed_world_camera_quat(
            orientation, self.wrist_view_pitch_deg, self.wrist_view_yaw_deg)
        for camera in self._wrist_pose_handles.values():
            camera.set_world_pose(orientation=aim_q, camera_axes="world")
        self._wrist_aimed = True

    def set_pose_2d(self, pose: Pose2d):
        self.articulation_view.set_velocities(np.zeros((1, 6)))
        pos = np.zeros(3)
        pos[0], pos[1], pos[2] = pose.x, pose.y, self.z_offset
        quat = np.asarray(euler_angles_to_quat(np.array([0.0, 0.0, pose.theta]))).flatten()
        self.articulation_view.set_world_poses(positions=pos[None, :], orientations=quat[None, :])
        self._command_pose = Pose2d(x=float(pose.x), y=float(pose.y), theta=float(pose.theta))

    def get_pose_2d(self) -> Pose2d:
        if self._command_pose is not None:
            return Pose2d(
                x=float(self._command_pose.x),
                y=float(self._command_pose.y),
                theta=float(self._command_pose.theta),
            )
        return self._physical_pose_2d()

    def _physical_pose_2d(self) -> Pose2d:
        pos, quat = self.articulation_view.get_world_poses()
        pos = np.asarray(pos).flatten()
        theta = quat_to_euler_angles(np.asarray(quat).flatten())[2]
        return Pose2d(x=float(pos[0]), y=float(pos[1]), theta=float(theta))
