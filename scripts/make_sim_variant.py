#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""生成 galbot_s1 的仿真派生副本 stages/galbot_s1_sim_src/。

对官方 usd/ 树做三处文本手术（原文件不动，副本可随时重新生成）:
1. payloads/Physics/physics.usda: 删除 root_joint 定义(把基座焊死在世界的 FixedJoint)
2. payloads/Physics/physx.usda: 删除对 root_joint 的 over; 把 PhysxArticulation 设置
   连同 PhysicsArticulationRootAPI 一起挂到机器人根 Xform —— 与官方 jetbot.usd 完全同构
3. 其余(payloads/geometries/纹理引用)原样保留
"""
import os
import re
import shutil

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(WS, "third_party", "galbot_s1_description", "usd")
DST = os.path.join(WS, "stages", "galbot_s1_sim_src")

if os.path.exists(DST):
    shutil.rmtree(DST)
shutil.copytree(SRC, DST)

# --- 1. physics.usda: 删 root_joint ---
p1 = os.path.join(DST, "payloads", "Physics", "physics.usda")
s = open(p1).read()
root_joint_def = '''    def PhysicsFixedJoint "root_joint" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        rel physics:body1 = </galbot_s1_v2_1_0_A21_ge103in/base_link>
    }

'''
assert root_joint_def in s, "physics.usda 中未找到 root_joint 定义(官方文件格式可能变了)"
s = s.replace(root_joint_def, "")
open(p1, "w").write(s)
print("1. 已删除 physics.usda 中的 root_joint 定义")

# --- 2. physx.usda: 根 Xform 挂 API, 删 root_joint over ---
p2 = os.path.join(DST, "payloads", "Physics", "physx.usda")
s = open(p2).read()

old_def = '''def "galbot_s1_v2_1_0_A21_ge103in"
{'''
new_def = '''def "galbot_s1_v2_1_0_A21_ge103in" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
)
{
    bool physxArticulation:enabledSelfCollisions = 0
    int physxArticulation:solverPositionIterationCount = 32
    int physxArticulation:solverVelocityIterationCount = 1'''
assert old_def in s, "physx.usda 根 Xform 定义未找到"
s = s.replace(old_def, new_def)

root_over = '''    over "root_joint" (
        prepend apiSchemas = ["PhysxArticulationAPI"]
    )
    {
        bool physxArticulation:enabledSelfCollisions = 0
        int physxArticulation:solverPositionIterationCount = 32
        int physxArticulation:solverVelocityIterationCount = 1
    }
'''
assert root_over in s, "physx.usda 中未找到 root_joint over"
s = s.replace(root_over, "")
open(p2, "w").write(s)
print("2. 已把 ArticulationRoot 移到根 Xform(仿 jetbot)并删除 root_joint over")


# --- 3. physx.usda: 轮关节 drive 修正烘焙 ---
# 官方驱动轮 damping=0.319(N·m·s/rad) 太弱推不动整机; 转向 k 折算 64506 过刚。
# steering: k=2(→114.6) d=0.2(→11.5); driving: k=0 d=3(→171.9); 限速放宽; maxForce 放开。
p3 = os.path.join(DST, "payloads", "Physics", "physx.usda")
s2 = open(p3).read()
import re as _re

def patch_joint(text, jname, max_vel, k, d):
    pat = _re.compile(
        r'(over "' + jname + r'" \([^)]*\)\s*\{)([^}]*)(\})'
    )
    m = pat.search(text)
    assert m, f"未找到 {jname} over 块"
    body = m.group(2)
    body = body.replace("float physxJoint:maxJointVelocity = 85.943665",
                        f"float physxJoint:maxJointVelocity = {max_vel}")
    body += f"""
        float drive:angular:physics:stiffness = {k}
        float drive:angular:physics:damping = {d}
        float drive:angular:physics:maxForce = 1000000000
"""
    return text[:m.start(2)] + body + text[m.end(2):]

for i in range(1, 5):
    s2 = patch_joint(s2, f"wheel_steering_joint{i}", "171.8874053955078", "2.0", "0.2")
    s2 = patch_joint(s2, f"wheel_driving_joint{i}", "1718.8740234375", "0.0", "3.0")
open(p3, "w").write(s2)
print("3. 轮关节 drive 修正已烘焙进 physx.usda")

# --- 4. 校验 ---
from pxr import Usd, UsdPhysics  # noqa: E402

stage = Usd.Stage.Open(os.path.join(DST, "galbot_s1.usda"))
dp = stage.GetDefaultPrim()
vss = dp.GetVariantSets()
vss.SetSelection("Robot", "robot")
vss.SetSelection("Sensor", "sensors")
vss.SetSelection("Physics", "physx")
rj = stage.GetPrimAtPath("/galbot_s1_v2_1_0_A21_ge103in/root_joint")
arts = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
print(f"3. 校验: root_joint defined={rj.IsDefined()} (over索引可残留但无关节本体) | ArticulationRoot={arts} | joints={len(joints)}")
assert not rj.IsDefined(), "root_joint 仍被定义!"
print("派生副本生成完成:", DST)
