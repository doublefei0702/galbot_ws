#!/usr/bin/env python3
"""Build an opt-in local map and reversible USD repair scene from one config.

Run with system python3 (pxr, PIL, numpy, PyYAML). The source USD is never edited.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
import yaml

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "exts/galbot.mobility_gen/galbot/mobility_gen"))
from local_workspace import load_workspace  # noqa: E402


def _asset_root(cfg: dict, source: Path) -> None:
    if "assets_dir" not in cfg:
        return  # USDZ carries its own assets.
    def ensure_link(link: Path, target: Path) -> None:
        if not target.is_dir():
            raise FileNotFoundError(f"USD Props assets unavailable: {target}")
        if link.is_symlink():
            if link.resolve() != target:
                raise ValueError(f"{link} points to another asset directory")
        elif not link.exists():
            link.symlink_to(target, target_is_directory=True)
        elif link.resolve() != target:
            raise ValueError(f"{link} already exists and is not the configured assets directory")

    props = Path(cfg["assets_dir"]).expanduser().resolve()
    ensure_link(source.parent / "Props", props)
    # Props' MDL and texture paths are ../Materials relative to the USD layer.
    ensure_link(source.parent / "Materials", props.parent / "Materials")
    # One forklift reference is ../../Props rather than ./Props in this USD.
    if "forklift_asset" in cfg:
        ensure_link(source.parent.parent.parent / "Props",
                    Path(cfg["forklift_asset"]).expanduser().resolve().parent.parent)


def _make_repair_layer(path: Path, cfg: dict) -> int:
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Repairs")
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, "/Repairs/Materials")
    count = 0
    for item in cfg.get("repairs", []):
        name = str(item["name"])
        if not name.isidentifier() or name in ("Materials",):
            raise ValueError(f"invalid repair prim name: {name}")
        if item["kind"] not in ("wall", "floor", "obstacle"):
            raise ValueError(f"invalid repair kind for {name}")
        if not item.get("purpose"):
            raise ValueError(f"repair {name} requires purpose")
        center = [float(v) for v in item["center_world"]]
        size = [float(v) for v in item["size_m"]]
        color = [float(v) for v in item["color_rgb"]]
        if len(center) != 3 or len(size) != 3 or len(color) != 3 or min(size) <= 0:
            raise ValueError(f"invalid geometry for {name}")
        if not item.get("render", False) and not item.get("collision", False):
            raise ValueError(f"repair {name} serves neither rendering nor collision")
        cube = UsdGeom.Cube.Define(stage, f"/Repairs/{name}")
        cube.GetSizeAttr().Set(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.GetPrim().CreateAttribute("userProperties:purpose", Sdf.ValueTypeNames.String).Set(str(item["purpose"]))
        cube.GetPrim().CreateAttribute("userProperties:kind", Sdf.ValueTypeNames.String).Set(str(item["kind"]))
        if item.get("collision", False):
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if item.get("render", False):
            material = UsdShade.Material.Define(stage, f"/Repairs/Materials/{name}")
            shader = UsdShade.Shader.Define(stage, f"/Repairs/Materials/{name}/PreviewSurface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
        else:
            cube.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        count += 1
    stage.GetRootLayer().Save()
    return count


def _make_scene(path: Path, source: Path, source_prim: Sdf.Path, repair: Path | None) -> None:
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    src = UsdGeom.Xform.Define(stage, "/World/WarehouseSource")
    src.GetPrim().GetReferences().AddReference(str(source), source_prim)
    if repair is not None:
        overlay = UsdGeom.Xform.Define(stage, "/World/LocalRepairs")
        overlay.GetPrim().GetReferences().AddReference(repair.name, Sdf.Path("/Repairs"))
    stage.GetRootLayer().Save()


def _grid_xy(x, y, origin, resolution):
    dx, dy = x - origin[0], y - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return ((c * dx + s * dy) / resolution, (-s * dx + c * dy) / resolution)


def _world_xy(col, row, origin, resolution):
    x, y = (col + 0.5) * resolution, (row + 0.5) * resolution
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return origin[0] + c * x - s * y, origin[1] + s * x + c * y


def _font(size):
    for p in ("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(p).is_file():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def _clip_height(poly: list[np.ndarray], height: float, keep_above: bool) -> list[np.ndarray]:
    result = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        inside_a = a[2] >= height if keep_above else a[2] <= height
        inside_b = b[2] >= height if keep_above else b[2] <= height
        if inside_a:
            result.append(a)
        if inside_a != inside_b:
            t = (height - a[2]) / (b[2] - a[2])
            result.append(a + t * (b - a))
    return result


def _raster_collision_height(cropped: np.ndarray, source_stage, cfg: dict,
                             origin, resolution, c0, r1) -> tuple[np.ndarray, int]:
    lower, upper = (float(v) for v in cfg["map_sampling_z_m"])
    if lower >= upper:
        raise ValueError("map_sampling_z_m must be [lower, upper]")
    image = Image.fromarray(cropped, mode="L")
    draw = ImageDraw.Draw(image)
    collision_meshes = 0
    # Candidate crop is tiny. Filter one large collision mesh vectorially, then
    # clip only nearby triangles to the robot-height slab before rasterizing.
    for prim in source_stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        collision_meshes += 1
        mesh = UsdGeom.Mesh(prim)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32)
        if not np.all(counts == 3):
            raise ValueError(f"collision mesh is not triangulated: {prim.GetPath()}")
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
        transform = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64)
        points = (np.column_stack((points, np.ones(len(points)))) @ transform)[:, :3]
        faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32).reshape(-1, 3)
        vertices = points[faces]
        xlo, xhi = min(p[0] for p in cfg["workspace_polygon_world"])-2, max(p[0] for p in cfg["workspace_polygon_world"])+2
        ylo, yhi = min(p[1] for p in cfg["workspace_polygon_world"])-2, max(p[1] for p in cfg["workspace_polygon_world"])+2
        near = ((vertices[:, :, 0].max(1) >= xlo) & (vertices[:, :, 0].min(1) <= xhi) &
                (vertices[:, :, 1].max(1) >= ylo) & (vertices[:, :, 1].min(1) <= yhi) &
                (vertices[:, :, 2].max(1) >= lower) & (vertices[:, :, 2].min(1) <= upper))
        for triangle in vertices[near]:
            polygon = _clip_height([v for v in triangle], lower, True)
            if polygon:
                polygon = _clip_height(polygon, upper, False)
            if len(polygon) < 2:
                continue
            pixels = []
            for vertex in polygon:
                col, row = _grid_xy(vertex[0], vertex[1], origin, resolution)
                pixels.append((round(col-c0), round(r1-1-row)))
            if len(pixels) >= 3:
                draw.polygon(pixels, fill=0)
            draw.line(pixels + pixels[:1], fill=0, width=2)
    return np.asarray(image), collision_meshes


def _make_map(output: Path, cfg: dict, workspace, source_stage) -> dict:
    map_path = Path(cfg["source_map"]).expanduser().resolve()
    source_cfg = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    source_img = Image.open(map_path.parent / source_cfg["image"]).convert("L")
    original = np.asarray(source_img)
    resolution = float(source_cfg["resolution"])
    origin = [float(v) for v in source_cfg["origin"]]
    points = [_grid_xy(x, y, origin, resolution) for x, y in workspace.polygon]
    pad = math.ceil(float(cfg.get("map_crop_padding_m", 1.0)) / resolution)
    c0 = max(0, math.floor(min(p[0] for p in points)) - pad)
    c1 = min(source_img.width, math.ceil(max(p[0] for p in points)) + pad)
    r0 = max(0, math.floor(min(p[1] for p in points)) - pad)
    r1 = min(source_img.height, math.ceil(max(p[1] for p in points)) + pad)
    if c1 <= c0 or r1 <= r0:
        raise ValueError("workspace is outside source map")
    cropped = original[source_img.height-r1:source_img.height-r0, c0:c1].copy()
    before = cropped.copy()
    height, width = cropped.shape
    # Retain source unknown gray cells. All cells outside the zone become
    # occupied (black), even if the source map labelled them white/free.
    for image_row in range(height):
        world_row = r1 - 1 - image_row
        for image_col in range(width):
            x, y = _world_xy(c0 + image_col, world_row, origin, resolution)
            if workspace.point_reason(x, y, clearance=0.0):
                cropped[image_row, image_col] = 0
    before_collision = cropped.copy()
    cropped, collision_meshes = _raster_collision_height(
        cropped, source_stage, cfg, origin, resolution, c0, r1)
    mesh_occupied_pixels = int(np.sum((before_collision != 0) & (cropped == 0)))
    map_image = output / "local_map.png"
    Image.fromarray(cropped, mode="L").save(map_image)
    new_origin = list(_world_xy(c0 - 0.5, r0 - 0.5, origin, resolution)) + [origin[2]]
    out_cfg = {"image": map_image.name, "resolution": resolution, "origin": new_origin,
               "negate": int(source_cfg.get("negate", 0)),
               "occupied_thresh": float(source_cfg.get("occupied_thresh", 0.65)),
               "free_thresh": float(source_cfg.get("free_thresh", 0.196))}
    (output / "local_map.yaml").write_text(yaml.safe_dump(out_cfg, sort_keys=False), encoding="utf-8")
    # Overlay is presentation only; local_map.png is the actual ROS input.
    preview_pad = math.ceil(5.0 / resolution)
    pc0, pc1 = max(0, c0-preview_pad), min(source_img.width, c1+preview_pad)
    pr0, pr1 = max(0, r0-preview_pad), min(source_img.height, r1+preview_pad)
    context = original[source_img.height-pr1:source_img.height-pr0, pc0:pc1]
    base = Image.fromarray(context).convert("RGB")
    base = base.resize((base.width*2, base.height*2), getattr(getattr(Image, "Resampling", Image), "NEAREST"))
    preview = Image.new("RGB", (base.width+240, base.height+120), "white")
    preview.paste(base, (60, 80))
    draw = ImageDraw.Draw(preview)
    def pxy(x, y):
        cc, rr = _grid_xy(x, y, origin, resolution)
        return round((cc-pc0)*2+60), round((pr1-rr)*2+80)
    draw.line([pxy(*p) for p in workspace.polygon] + [pxy(*workspace.polygon[0])], fill=(240, 30, 30), width=5)
    for polygon in workspace.excludes:
        draw.line([pxy(*p) for p in polygon] + [pxy(*polygon[0])], fill=(255, 160, 0), width=5)
    for i, (x, y) in enumerate(workspace.polygon):
        px, py = pxy(x, y)
        draw.ellipse((px-5, py-5, px+5, py+5), fill=(255, 0, 0))
        draw.text((px+6, py+5), f"{i}: ({x:g}, {y:g})", fill=(255, 0, 0), font=_font(16), stroke_width=2, stroke_fill="white")
    start = cfg["initial_pose_world"]
    sx, sy = pxy(float(start[0]), float(start[1]))
    draw.ellipse((sx-7, sy-7, sx+7, sy+7), fill=(0, 70, 255))
    draw.text((12, 12), "候选区域 / CANDIDATE AREA", fill=(255, 0, 0), font=_font(22), stroke_width=2, stroke_fill="white")
    draw.text((12, 42), f"Blue: candidate robot start ({start[0]:g}, {start[1]:g})", fill=(0, 40, 200), font=_font(16), stroke_width=2, stroke_fill="white")
    preview.save(output / "candidate_preview.png")
    return {"source_map": str(map_path), "source_image_shape": list(original.shape),
            "crop_grid": [c0, r0, c1, r1], "local_image_shape": [height, width],
            "free_cells": int(np.sum(cropped >= 205)),
            "occupied_cells": int(np.sum(cropped <= 89)), "origin": new_origin,
            "collision_height_band_m": [float(v) for v in cfg["map_sampling_z_m"]],
            "collision_mesh_count": collision_meshes,
            "newly_occupied_by_collision_mesh": mesh_occupied_pixels}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WS / "configs/scene_with_collision_local.yaml")
    parser.add_argument("--output", type=Path, default=Path("/root/gpufree-data/data_usd/scene_local_candidate"))
    args = parser.parse_args()
    workspace = load_workspace(args.workspace)
    cfg = yaml.safe_load(workspace.source.read_text(encoding="utf-8"))
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = Path(cfg["source_usd"]).expanduser().resolve()
    _asset_root(cfg, source)
    if not source.is_file():
        raise FileNotFoundError(source)
    source_stage = Usd.Stage.Open(str(source))
    source_prim = source_stage.GetDefaultPrim()
    if not source_prim.IsValid():
        raise ValueError(f"source USD has no default prim: {source}")
    if (UsdGeom.GetStageMetersPerUnit(source_stage) != 1.0 or
            UsdGeom.GetStageUpAxis(source_stage) != UsdGeom.Tokens.z):
        raise ValueError("workspace builder currently requires a meter/Z-up source stage")
    repair = output / "repair_layer.usd"
    count = _make_repair_layer(repair, cfg)
    _make_scene(output / "local_work_scene.usd", source, source_prim.GetPath(), repair)
    _make_scene(output / "local_source_only.usd", source, source_prim.GetPath(), None)
    result = _make_map(output, cfg, workspace, source_stage)
    nav_source = Path(cfg["source_nav_config"]).expanduser().resolve()
    nav_cfg = yaml.safe_load(nav_source.read_text(encoding="utf-8"))
    nav_cfg["galbot_navigator"]["ros__parameters"]["footprint_radius"] = workspace.clearance_m
    (output / "navigation_local.yaml").write_text(
        yaml.safe_dump(nav_cfg, sort_keys=False), encoding="utf-8")
    result.update({"status": cfg["status"], "repair_prim_count": count,
                   "source_usd": str(source), "workspace_config": str(workspace.source),
                   "navigation_footprint_radius_m": workspace.clearance_m})
    (output / "build_report.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
