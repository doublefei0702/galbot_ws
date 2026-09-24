# SPDX-License-Identifier: Apache-2.0
"""Exact planar base motion for ROS body-frame Twist commands."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlanarBaseController:
    """Integrate ``[vx, vy, wz]`` as one rigid body in the SE(2) plane."""

    @staticmethod
    def integrate(
        x: float,
        y: float,
        yaw: float,
        command: np.ndarray,
        step_size: float,
    ) -> tuple[float, float, float]:
        values = np.asarray(command, dtype=float).reshape(-1)
        if values.size == 2:
            vx, wz = values
            vy = 0.0
        elif values.size == 3:
            vx, vy, wz = values
        else:
            raise ValueError(f"command must contain 2 or 3 values, got {values.size}")
        dt = float(step_size)
        if dt <= 0.0:
            raise ValueError(f"step_size must be positive, got {step_size}")

        theta = float(wz) * dt
        # Constant body-frame twist integrated exactly over one physics step.
        if abs(theta) < 1e-8:
            body_dx = float(vx) * dt
            body_dy = float(vy) * dt
        else:
            sin_theta, cos_theta = math.sin(theta), math.cos(theta)
            body_dx = (sin_theta * vx - (1.0 - cos_theta) * vy) / wz
            body_dy = ((1.0 - cos_theta) * vx + sin_theta * vy) / wz
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return (
            float(x) + cos_yaw * body_dx - sin_yaw * body_dy,
            float(y) + sin_yaw * body_dx + cos_yaw * body_dy,
            (float(yaw) + math.pi + theta) % (2.0 * math.pi) - math.pi,
        )
