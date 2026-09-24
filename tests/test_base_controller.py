import math
import importlib.util
import os
from pathlib import Path
import sys

import numpy as np

MODULE = os.path.join(os.path.dirname(__file__), "..", "exts", "galbot.mobility_gen",
                      "galbot", "mobility_gen", "base_controller.py")
spec = importlib.util.spec_from_file_location("base_controller", MODULE)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
PlanarBaseController = module.PlanarBaseController


def test_body_frame_translation_at_yaw_90():
    pose = PlanarBaseController.integrate(
        1.0, -2.0, math.pi / 2.0, np.array([0.2, 0.0, 0.0]), 0.5)
    assert pose == (1.0, -1.9, math.pi / 2.0)


def test_lateral_command_uses_body_frame():
    pose = PlanarBaseController.integrate(
        0.0, 0.0, math.pi / 2.0, np.array([0.0, 0.3, 0.0]), 1.0)
    assert np.allclose(pose, (-0.3, 0.0, math.pi / 2.0))


def test_arc_integration_is_exact_for_constant_twist():
    pose = PlanarBaseController.integrate(
        0.0, 0.0, 0.0, np.array([1.0, 0.0, 1.0]), math.pi / 2.0)
    assert np.allclose(pose, (1.0, 1.0, math.pi / 2.0))


def test_legacy_two_value_command_is_forward_and_yaw():
    pose = PlanarBaseController.integrate(
        0.0, 0.0, 0.0, np.array([1.0, 0.5]), 1.0)
    expected = (math.sin(0.5) / 0.5, (1.0 - math.cos(0.5)) / 0.5, 0.5)
    assert np.allclose(pose, expected)
