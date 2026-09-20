# SPDX-License-Identifier: Apache-2.0
"""Galbot S1 四轮独立转向 (swerve/steer-drive) 底盘运动学控制器。

底盘为 4 个模块：每个模块一个转向关节(revolute, 绕Z) + 一个驱动关节(continuous)。
几何参数来自 USD 实测 (wheel_steering_joint1..4 的 physics:localPos0):
    joint1: (+0.247, -0.247)  前右 FR
    joint2: (+0.247, +0.247)  前左 FL
    joint3: (-0.247, -0.247)  后右 RR
    joint4: (-0.247, +0.247)  后左 RL
约定: +x 前, +y 左, theta 绕 +z 逆时针为正 (REP-103)。

纯 numpy 实现，不依赖 Isaac Sim，可用 tests/test_swerve_controller.py 离线验证。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# 转向关节限位 (rad), URDF/USD 均为 ±166.88°
STEERING_LIMIT_RAD = math.radians(166.88)


@dataclass
class SwerveModuleGeometry:
    name: str
    x: float
    y: float
    drive_sign: float = 1.0  # 正向驱动时该轮驱动关节角速度符号(实测标定)
    steering_offset_rad: float = 0.0  # 转向零位偏移(实测标定)


DEFAULT_MODULES = [
    SwerveModuleGeometry("wheel_1_fr", +0.247, -0.247),
    SwerveModuleGeometry("wheel_2_fl", +0.247, +0.247),
    SwerveModuleGeometry("wheel_3_rr", -0.247, -0.247),
    SwerveModuleGeometry("wheel_4_rl", -0.247, +0.247),
]


def _normalize_angle(a: float) -> float:
    """归一化到 [-pi, pi]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class SwerveController:
    """输入 chassis twist (vx, vy, wz)，输出每个模块的 (转向角, 轮速 rad/s)。

    wheel_radius TODO: 待在 Isaac Sim 中用轮网格实测(暂用 0.08 占位)。
    """

    wheel_radius: float = 0.08
    modules: list = field(default_factory=lambda: list(DEFAULT_MODULES))
    steer_speed_limit: float = 1.5  # rad/s, physxJoint:maxJointVelocity
    wheel_speed_limit: float = 30.0  # rad/s 占位，待实测

    def forward(self, command: np.ndarray) -> list:
        """command = [vx, vy, wz] (m/s, m/s, rad/s)。返回 [(steer_rad, wheel_rad_speed), ...]。"""
        vx, vy, wz = float(command[0]), float(command[1]), float(command[2])
        out = []
        for m in self.modules:
            wheel_vx = vx - wz * m.y
            wheel_vy = vy + wz * m.x
            speed = math.hypot(wheel_vx, wheel_vy)
            if speed < 1e-6:
                # 零速: 保持当前转向角, 轮速置零(由调用方传入当前角)
                out.append((None, 0.0))
                continue
            steer = math.atan2(wheel_vy, wheel_vx)
            wheel_w = speed / self.wheel_radius * m.drive_sign
            out.append((steer, wheel_w))
        return out

    def forward_with_state(
        self, command: np.ndarray, current_steers: list
    ) -> list:
        """带状态的版本: 处理零速保持、>90° 翻转、限位。

        current_steers: 每个模块当前转向角 (rad)。
        返回 [(steer_target_rad, wheel_omega_rad_s), ...]
        """
        vx, vy, wz = float(command[0]), float(command[1]), float(command[2])
        results = []
        for m, cur in zip(self.modules, current_steers):
            wheel_vx = vx - wz * m.y
            wheel_vy = vy + wz * m.x
            speed = math.hypot(wheel_vx, wheel_vy)
            if speed < 1e-6:
                results.append((cur, 0.0))
                continue
            steer = math.atan2(wheel_vy, wheel_vx)
            wheel_w = speed / self.wheel_radius * m.drive_sign

            # 翻转优化: 若目标角与当前角差超过 90°, 翻转 180° 并反转轮速
            delta = _normalize_angle(steer - cur)
            if abs(delta) > math.pi / 2:
                steer = _normalize_angle(steer + math.pi)
                wheel_w = -wheel_w
                delta = _normalize_angle(steer - cur)

            # 限位保护(转向 ±166.88°, 物理上几乎不会触界)
            steer = max(-STEERING_LIMIT_RAD, min(STEERING_LIMIT_RAD, steer + m.steering_offset_rad))
            results.append((steer, wheel_w))
        return results
