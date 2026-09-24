#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""预生成 仓库+Galbot 组合采集场景 stages/galbot_warehouse_capture.usd。

与 galbot_s1_debug.usd 同构(该模式的驱动写入已验证可用):
- /World ← 仓库(绝对路径引用)
- /World/robot ← galbot 仿真变体(变体选择写死在文件里) + 出生位姿
- 不放 PhysicsScene (由 World() 创建)

用法: python3 scripts/build_capture_stage.py [x y theta_deg]
"""
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WAREHOUSE = "/root/gpufree-data/IsaacSim/IsaacSim/Galbot_test/Simple_Warehouse/warehouse_with_forklifts.usd"
GALBOT = os.path.join(WS, "stages", "galbot_s1_sim_src", "galbot_s1.usda")
DST = os.path.join(WS, "stages", "galbot_warehouse_capture.usd")

x, y, theta_deg = (float(v) for v in (sys.argv[1:4] if len(sys.argv) > 3 else (0.5, -5.0, 0.0)))

stage = Usd.Stage.CreateNew(DST)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# 仓库
wr = world.GetPrim().GetReferences()
wr.AddReference(Sdf.Reference(assetPath=WAREHOUSE, primPath="/World"))

# 机器人
robot = UsdGeom.Xform.Define(stage, "/World/robot")
robot.GetPrim().GetReferences().AddReference(
    Sdf.Reference(assetPath=GALBOT, primPath="/galbot_s1_v2_1_0_A21_ge103in"))
vs = robot.GetPrim().GetVariantSets()
vs.SetSelection("Robot", "robot")
vs.SetSelection("Sensor", "sensors")
vs.SetSelection("Physics", "physx")
t_op = robot.GetTranslateOp()
if t_op:
    t_op.Set(Gf.Vec3d(x, y, 0.05))
else:
    robot.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.05))
r_op = robot.GetRotateXYZOp()
if r_op:
    r_op.Set(Gf.Vec3f(0.0, 0.0, theta_deg))
else:
    # Xform op order [translate, rotateXYZ] gives a local yaw followed by the
    # requested world translation, so initial x/y are not rotated about origin.
    robot.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, theta_deg))

stage.GetRootLayer().Save()
print(f"已生成 {DST} 出生点=({x},{y},{theta_deg}°)")

# 自检
check = Usd.Stage.Open(DST)
joints = sum(1 for p in check.Traverse() if p.IsA(UsdPhysics.Joint))
sel = check.GetPrimAtPath("/World/robot").GetVariantSets().GetAllVariantSelections()
print(f"自检: joints={joints} 变体={sel}")
