#!/usr/bin/env python3
"""非破坏性检查 Galbot S1 官方 USD：单位、变体、关节、驱动、碰撞、相机挂点。

用法（系统 Python 即可，依赖 pip 的 usd-core）:
    python3 scripts/inspect_galbot_s1.py [USD路径]

输出: stages/galbot_s1_inspection.md
"""
import os
import sys

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

USD_PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__), "..", "third_party", "galbot_s1_description", "usd", "galbot_s1.usda"
)
REPORT = os.path.join(os.path.dirname(__file__), "..", "stages", "galbot_s1_inspection.md")


def fmt(v):
    if v is None:
        return "-"
    return ", ".join(f"{x:.4g}" if isinstance(x, float) else str(x) for x in (v if isinstance(v, (list, tuple)) else [v]))


def main():
    lines = []
    stage = Usd.Stage.Open(USD_PATH)
    if not stage:
        print("无法打开:", USD_PATH)
        sys.exit(1)

    rl = stage.GetRootLayer()
    lines.append(f"# Galbot S1 USD 检查报告\n")
    lines.append(f"- 文件: `{os.path.abspath(USD_PATH)}`")
    lines.append(f"- metersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")
    lines.append(f"- upAxis: {UsdGeom.GetStageUpAxis(stage)}")
    lines.append(f"- defaultPrim: {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else '(无)'}")

    # 变体选择（默认 Robot=none 只有骨架，切到完整组合再统计）
    dp = stage.GetDefaultPrim()
    vss = dp.GetVariantSets()
    if vss.GetVariantSelection("Robot") == "none":
        vss.SetSelection("Robot", "robot")
    if vss.GetVariantSelection("Physics") == "none":
        vss.SetSelection("Physics", "physx")
    dp = stage.GetDefaultPrim()
    lines.append("\n## 变体 (variantSets)\n")
    for vs in dp.GetVariantSets().GetNames():
        lines.append(f"- {vs} = {dp.GetVariantSet(vs).GetVariantSelection()}")

    # Articulation Root
    lines.append("\n## Articulation Root\n")
    art_roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    for p in art_roots:
        lines.append(f"- {p.GetPath()}")
    if not art_roots:
        lines.append("- (未找到!)")

    # 关节
    lines.append("\n## 关节 (名称 | 类型 | 轴 | 下限 | 上限 | Drive类型 | stiffness | damping | maxForce | maxVelocity)\n")
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    n_movable = 0
    for j in joints:
        jt = j.GetPrimTypeInfo().GetTypeName()
        jname = j.GetName()
        axis = "-"
        lo = hi = None
        for attr in ("physics:lowerLimit", "physics:upperLimit"):
            pass
        lo_attr = j.GetAttribute("physics:lowerLimit")
        hi_attr = j.GetAttribute("physics:upperLimit")
        if lo_attr and lo_attr.HasValue():
            lo = lo_attr.Get()
        if hi_attr and hi_attr.HasValue():
            hi = hi_attr.Get()
        if jt != "PhysicsFixed":
            n_movable += 1
        # drive 信息挂在关节下的 Drive 子 prim 或 API 上
        drive_info = ""
        for child in j.GetChildren():
            cty = child.GetTypeName()
            if "Drive" in cty:
                st = child.GetAttribute("physics:stiffness")
                da = child.GetAttribute("physics:damping")
                mf = child.GetAttribute("physics:maxForce")
                mv = child.GetAttribute("physics:maxVelocity")
                tg = child.GetAttribute("physics:target")
                tv = child.GetAttribute("physics:targetVelocity")
                drive_info = (f"drive={cty.replace('Physics','').replace('Drive','')} "
                              f"target={fmt(tg.Get() if tg and tg.HasValue() else None)} "
                              f"targetVel={fmt(tv.Get() if tv and tv.HasValue() else None)} "
                              f"k={fmt(st.Get() if st and st.HasValue() else None)} "
                              f"d={fmt(da.Get() if da and da.HasValue() else None)} "
                              f"maxF={fmt(mf.Get() if mf and mf.HasValue() else None)}")
        lines.append(f"- `{jname}` | {jt.replace('Physics','')} | lo={fmt(lo)} hi={fmt(hi)} | {drive_info}")
    lines.append(f"\n可动关节总数: {n_movable}")

    # 物理
    n_coll = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
    n_mass = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.MassAPI))
    n_mesh = sum(1 for p in stage.Traverse() if p.GetTypeName() in ("Mesh", "Capsule", "Sphere", "Cube", "Cylinder", "Cone"))
    # 网格位于 instanceable 原型中，需跨原型统计
    proto_mesh = proto_coll = 0
    for proto in stage.GetPrototypes():
        stack = [proto]
        while stack:
            q = stack.pop()
            if q.GetTypeName() == "Mesh":
                proto_mesh += 1
            if q.HasAPI(UsdPhysics.CollisionAPI):
                proto_coll += 1
            stack.extend(q.GetChildren())
    n_mesh += proto_mesh
    n_coll += proto_coll
    lines.append(f"\n## 物理\n")
    lines.append(f"- CollisionAPI prims: {n_coll}")
    lines.append(f"- MassAPI prims: {n_mass}")
    scenes = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"]
    lines.append(f"- PhysicsScene: {[str(p.GetPath()) for p in scenes] or '(无)'}")
    lines.append(f"- 几何 prims (Mesh/Cube/...): {n_mesh}")

    # 相机
    lines.append("\n## 相机挂点\n")
    for p in stage.Traverse():
        if p.GetTypeName() == "Camera":
            lines.append(f"- Camera prim: {p.GetPath()}")
    for p in stage.Traverse():
        if "camera_link" in p.GetName() and "visuals" not in str(p.GetPath()) and "collisions" not in str(p.GetPath()):
            lines.append(f"- {p.GetName()}: {p.GetPath()}")

    # 关键轮关节存在性
    lines.append("\n## 轮关节核对\n")
    names = {j.GetName() for j in joints}
    for i in range(1, 5):
        for pat in (f"wheel_steering_joint{i}", f"wheel_driving_joint{i}"):
            lines.append(f"- {pat}: {'OK' if pat in names else '缺失!'}")

    report = "\n".join(lines)
    os.makedirs(os.path.dirname(os.path.abspath(REPORT)), exist_ok=True)
    with open(REPORT, "w") as f:
        f.write(report + "\n")
    print(report)


if __name__ == "__main__":
    main()
