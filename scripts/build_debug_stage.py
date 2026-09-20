#!/usr/bin/env python3
"""生成非破坏性 Galbot S1 调试场景 stages/galbot_s1_debug.usd。

内容:
- 引用官方 galbot_s1.usda（不修改原文件）
- 显式选择变体 Robot=robot / Sensor=sensors / Physics=physx
  （官方默认是 Robot=none，只有骨架没有几何和碰撞）
- 地面(带碰撞)、DistantLight、PhysicsScene
- 机器人初始抬高 0.15m 便于下落自检

用法: python3 scripts/build_debug_stage.py   (依赖 pip 的 usd-core)
"""
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(WS, "stages", "galbot_s1_sim_src", "galbot_s1.usda")
DST = os.path.join(WS, "stages", "galbot_s1_debug.usd")
ROBOT_PRIM = "/World/robot"


def main():
    stage = Usd.Stage.CreateNew(DST)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # 注意: 不在这里定义 PhysicsScene —— World()/SimulationContext 会创建自己的物理场景，
    # 两个场景会导致 "Physics scenes stepping is not the same" 错误

    # --- 地面 ---
    ground = UsdGeom.Mesh.Define(stage, "/World/ground")
    # Plane 表面
    ground.CreatePointsAttr(
        [Gf.Vec3f(-100, -100, 0), Gf.Vec3f(100, -100, 0), Gf.Vec3f(100, 100, 0), Gf.Vec3f(-100, 100, 0)]
    )
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDoubleSidedAttr(True)
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # --- 灯光: pip usd-core 无 UsdLux, 用裸类型 prim 授权(kit 加载时解析) ---
    dome = stage.DefinePrim("/World/domeLight", "DomeLight")
    dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)
    sun = stage.DefinePrim("/World/sunLight", "DistantLight")
    sun.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1500.0)
    sun.CreateAttribute("xformOp:rotateX", Sdf.ValueTypeNames.Float).Set(-55.0)
    sun.CreateAttribute("xformOp:rotateZ", Sdf.ValueTypeNames.Float).Set(-25.0)
    sun.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(["xformOp:rotateX", "xformOp:rotateZ"])

    # --- 机器人（引用 + 变体选择，全部写在包装层） ---
    robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM)
    refs = robot.GetPrim().GetReferences()
    refs.AddReference(Sdf.Reference(assetPath=SRC, primPath="/galbot_s1_v2_1_0_A21_ge103in"))
    vs = robot.GetPrim().GetVariantSets()
    vs.SetSelection("Robot", "robot")
    vs.SetSelection("Sensor", "sensors")
    vs.SetSelection("Physics", "physx")
    t_op = robot.GetTranslateOp()
    if t_op:
        t_op.Set(Gf.Vec3d(0, 0, 0.15))
    else:
        robot.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.15))


    # 轮关节 drive 覆盖(包装层): 官方增益(转向 k=1125/deg → PhysX 64506 N·m/rad)
    # 过刚导致求解器无响应; 改用 jetbot 同款 sane 值。
    # 注意 USD 角度 drive 增益按"每度"authored, PhysX 内部乘 57.3 转每弧度。
    for i in range(1, 5):
        for kind, k, d in (("wheel_steering", 2.0, 0.2), ("wheel_driving", 0.0, 3.0)):
            jp = stage.OverridePrim(f"{ROBOT_PRIM}/joints/{kind}_joint{i}")
            drive = UsdPhysics.DriveAPI(jp, "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k)
            drive.CreateDampingAttr(d)
            drive.CreateMaxForceAttr(1e9)
            # 官方 physxJoint:maxJointVelocity=85.94(度/s)=1.5rad/s 把轮子限死
            # (轮缘仅0.12m/s)。放宽: 驱动30rad/s、转向3rad/s (单位:度/s)
            # PhysxSchema 不在开源 usd-core 里, 直接裸author属性(PhysX 识别)
            max_deg = 3.0 * 57.2958 if kind == "wheel_steering" else 30.0 * 57.2958
            attr = jp.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float)
            attr.Set(max_deg)

    stage.GetRootLayer().Save()
    print("已生成:", DST)

    # --- 自检 ---
    check = Usd.Stage.Open(DST)
    joints = [p for p in check.Traverse() if p.IsA(UsdPhysics.Joint)]
    n_proto = len(check.GetPrototypes())
    n_mesh = 0
    n_coll = 0
    for proto in check.GetPrototypes():
        stack = [proto]
        while stack:
            p = stack.pop()
            if p.GetTypeName() == "Mesh":
                n_mesh += 1
            if p.HasAPI(UsdPhysics.CollisionAPI):
                n_coll += 1
            stack.extend(p.GetChildren())
    cam_links = [str(p.GetPath()) for p in check.Traverse() if "camera_link" in p.GetName()]
    art = [str(p.GetPath()) for p in check.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print(f"自检: joints={len(joints)}, prototypes={n_proto}, Mesh(原型内)={n_mesh}, Collision(原型内)={n_coll}")
    print(f"      articulationRoot={art}")
    print(f"      camera_link={cam_links}")
    sel = check.GetPrimAtPath(ROBOT_PRIM).GetVariantSets().GetAllVariantSelections()
    print(f"      变体={sel}")


if __name__ == "__main__":
    main()
