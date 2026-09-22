#!/usr/bin/env python3
"""Offline tests for wrist-camera axis conventions (no Isaac Sim required)."""
import importlib.util
import os

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODULE = os.path.join(
    ROOT, "exts", "galbot.mobility_gen", "galbot", "mobility_gen", "pose_utils.py")
spec = importlib.util.spec_from_file_location("galbot_pose_utils", MODULE)
pose_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose_utils)


def test_navigation_pitch():
    quat = pose_utils.aimed_world_camera_quat(np.array([1.0, 0.0, 0.0, 0.0]), -10.0, 0.0)
    forward, pitch = pose_utils.camera_forward_pitch(quat)
    assert np.allclose(forward, [np.cos(np.deg2rad(10)), 0.0, -np.sin(np.deg2rad(10))])
    assert abs(pitch + 10.0) < 1e-9


def test_view_is_relative_to_base():
    base_yaw = np.deg2rad(90.0)
    base_quat = np.array([np.cos(base_yaw / 2), 0.0, 0.0, np.sin(base_yaw / 2)])
    quat = pose_utils.aimed_world_camera_quat(base_quat, -10.0, 0.0)
    forward, pitch = pose_utils.camera_forward_pitch(quat)
    assert np.allclose(forward, [0.0, np.cos(np.deg2rad(10)), -np.sin(np.deg2rad(10))], atol=1e-9)
    assert abs(pitch + 10.0) < 1e-9


def test_rotation_round_trip():
    quat = np.array([0.7, -0.2, 0.3, 0.6], dtype=float)
    quat /= np.linalg.norm(quat)
    recovered = pose_utils.matrix_to_quat_wxyz(pose_utils.quat_wxyz_to_matrix(quat))
    assert abs(float(np.dot(quat, recovered))) > 1.0 - 1e-9  # q and -q are equivalent


def test_horizontal_fov_to_focal_length():
    aperture = 2.0955
    focal = pose_utils.focal_length_for_horizontal_fov(aperture, 70.0)
    recovered_fov = np.degrees(2.0 * np.arctan(aperture / (2.0 * focal)))
    assert abs(recovered_fov - 70.0) < 1e-9


if __name__ == "__main__":
    test_navigation_pitch()
    test_view_is_relative_to_base()
    test_rotation_round_trip()
    test_horizontal_fov_to_focal_length()
    print("camera pose tests passed")
