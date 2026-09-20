# SPDX-License-Identifier: Apache-2.0
"""坐标系与相机约定工具。

Isaac Sim 5 ``Camera`` API 有三套轴约定，不能混用：

* ``camera_axes="world"``: +X 前、+Z 上（机器人常用）；
* ``camera_axes="ros"``: +Z 前、+X 右、+Y 下（OpenCV optical）；
* ``camera_axes="usd"``: -Z 前、+Y 上（USD Camera prim 原生约定）。

创建/读取相机时显式传 ``camera_axes``。旧代码把默认的 ``world`` 返回值当成
USD 位姿又转换了一次，这是腕部画面方向错误的主要原因。
"""
from __future__ import annotations

import numpy as np

USD_CAMERA_FROM_OPENCV = np.diag([1.0, -1.0, -1.0, 1.0])
OPENCV_FROM_USD_CAMERA = np.diag([1.0, -1.0, -1.0, 1.0])


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    """Scalar-first quaternion to a 3x3 rotation matrix."""
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix to a normalized scalar-first quaternion."""
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                          (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
        elif i == 1:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                          0.25 * s, (m[1, 2] + m[2, 1]) / s])
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                          (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def aimed_world_camera_quat(base_quat_wxyz: np.ndarray, pitch_deg: float = -10.0,
                            yaw_deg: float = 0.0) -> np.ndarray:
    """Return a ``camera_axes='world'`` orientation relative to robot base.

    The camera's +X axis is its forward direction and +Z is image-up.  Negative
    pitch looks down; positive yaw looks left.  Roll is deliberately fixed at
    zero so the horizon cannot become diagonal.
    """
    pitch = np.deg2rad(float(pitch_deg))
    yaw = np.deg2rad(float(yaw_deg))
    cy, sy = np.cos(yaw), np.sin(yaw)
    # A conventional negative pitch must rotate +X toward -Z, hence Ry(-pitch).
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return matrix_to_quat_wxyz(quat_wxyz_to_matrix(base_quat_wxyz) @ rz @ ry)


def camera_forward_pitch(quat_wxyz: np.ndarray) -> tuple[np.ndarray, float]:
    """Forward vector and pitch for a ``camera_axes='world'`` quaternion."""
    forward = quat_wxyz_to_matrix(quat_wxyz)[:, 0]
    pitch = np.degrees(np.arctan2(forward[2], np.hypot(forward[0], forward[1])))
    return forward, float(pitch)


def pose_matrix(position: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """Build a homogeneous pose matrix from position and scalar-first quat."""
    transform = np.eye(4)
    transform[:3, :3] = quat_wxyz_to_matrix(quat_wxyz)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def world_from_opencv_camera(world_from_usd_camera: np.ndarray) -> np.ndarray:
    return world_from_usd_camera @ USD_CAMERA_FROM_OPENCV


def opencv_camera_from_world(world_from_usd_camera: np.ndarray) -> np.ndarray:
    """T_cv_world: 把世界点变换到 OpenCV 相机系。"""
    return np.linalg.inv(world_from_opencv_camera(world_from_usd_camera))


def make_camera_info(k: np.ndarray, width: int, height: int, depth_scale: float = 1000.0) -> dict:
    """生成 ROS CameraInfo 风格的字典（K 来自实际渲染读取，而非配置）。"""
    return {
        "width": width,
        "height": height,
        "K": k.tolist(),
        "model": "plumb_bob",
        "depth_scale": depth_scale,
        "distortion_model": "rational_polynomial",
        "D": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
