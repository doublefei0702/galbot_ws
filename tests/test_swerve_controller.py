#!/usr/bin/env python3
"""swerve 控制器离线几何验证（不需要 Isaac Sim）。

覆盖用户计划中的验收项:
- 直行: 四轮转角≈0, 轮速一致
- 后退: 轮速取反
- 原地旋转: 四轮转角指向切线方向, 转速一致
- 固定半径转弯: 几何关系 v_i = vx - wz*y_i (vy 分量)
- 停止: 轮速全零且保持转向角
- 翻转优化: 目标角差>90°时翻转
"""
import math
import importlib.util
import os
import sys

import numpy as np

MODULE = os.path.join(os.path.dirname(__file__), "..", "exts", "galbot.mobility_gen",
                      "galbot", "mobility_gen", "swerve_controller.py")
spec = importlib.util.spec_from_file_location("swerve_controller", MODULE)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
SwerveController = module.SwerveController

R = 0.08


def approx(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_straight():
    c = SwerveController(wheel_radius=R)
    out = c.forward(np.array([1.0, 0.0, 0.0]))
    for module_geometry, (steer, w) in zip(c.modules, out):
        assert approx(steer, 0.0, 1e-9), steer
        assert approx(w, module_geometry.drive_sign / R), w
    print("PASS 直行: 四轮转角=0, 物理驱动符号已标定")


def test_backward():
    c = SwerveController(wheel_radius=R)
    out = c.forward(np.array([-1.0, 0.0, 0.0]))
    for module_geometry, (steer, w) in zip(c.modules, out):
        # 转向 180° + 正转速 == 倒退 (与 转向0 + 负转速 等价)
        assert approx(abs(steer), math.pi), steer
        assert approx(w, module_geometry.drive_sign / R), w
    print("PASS 后退: 转角=180°, 正转速滚动等效倒退")


def test_spin():
    c = SwerveController(wheel_radius=R)
    out = c.forward(np.array([0.0, 0.0, 1.0]))  # 逆时针
    # 期望: 每轮指向自身位置旋转90°的切线方向 = atan2(x, -y)
    for m, (steer, w) in zip(c.modules, out):
        expect = math.atan2(m.x, -m.y)
        assert approx(_norm(steer), _norm(expect), 1e-9), (m.name, steer, expect)
        assert approx(w, m.drive_sign * 0.247 * math.sqrt(2) / R), w
    print("PASS 原地旋转: 四轮切向, |v|=wz*0.349m")


def _norm(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def test_fixed_radius_turn():
    c = SwerveController(wheel_radius=R)
    vx, wz = 0.5, 0.25
    out = c.forward(np.array([vx, 0.0, wz]))
    for m, (steer, w) in zip(c.modules, out):
        wx = vx - wz * m.y
        wy = wz * m.x
        assert approx(_norm(steer), _norm(math.atan2(wy, wx)), 1e-9)
        assert approx(w, m.drive_sign * math.hypot(wx, wy) / R), w
    print("PASS 固定半径转弯: 满足 v_i = vx - wz*y_i 几何关系")


def test_stop_hold():
    c = SwerveController(wheel_radius=R)
    cur = [0.1, 0.2, -0.3, 0.0]
    out = c.forward_with_state(np.array([0.0, 0.0, 0.0]), cur)
    for (steer, w), c0 in zip(out, cur):
        assert steer == c0  # 保持原转向角
        assert w == 0.0
    print("PASS 停止: 轮速归零, 转向角保持")


def test_flip_optimization():
    c = SwerveController(wheel_radius=R)
    cur = [0.0] * 4
    out = c.forward_with_state(np.array([-0.1, 0.0, 0.0]), cur)  # 要求 180°
    for module_geometry, (steer, w) in zip(c.modules, out):
        # 翻转后目标角接近 0 而不是 ±pi, 物理轮速相对前进反号。
        assert abs(_norm(steer)) < 0.01, steer
        assert w * module_geometry.drive_sign < 0
    print("PASS 翻转优化: >90° 行程时翻转轮速")


if __name__ == "__main__":
    test_straight()
    test_backward()
    test_spin()
    test_fixed_radius_turn()
    test_stop_hold()
    test_flip_optimization()
    print("\n全部通过 ✔")
