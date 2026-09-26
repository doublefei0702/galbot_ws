#!/usr/bin/env python3
"""Make a before/after contact sheet and numerical depth comparison."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--before", type=Path, required=True)
parser.add_argument("--after", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--view", help="render just one configured view")
parser.add_argument("--include-mask", action="store_true", help="also show the valid-depth mask")
args = parser.parse_args()

views = tuple(item["view"] for item in json.loads((args.before/"summary.json").read_text())["views"])
if args.view:
    if args.view not in views:
        parser.error(f"unknown view: {args.view}")
    views = (args.view,)
rows_per_view = 3 if args.include_mask else 2
canvas = Image.new("RGB", (640*2, (480*rows_per_view+35)*len(views)), "white")
draw = ImageDraw.Draw(canvas)
report = []
for i, name in enumerate(views):
    before = np.load(args.before/name/"depth_raw.npy")
    after = np.load(args.after/name/"depth_raw.npy")
    bmask = np.asarray(Image.open(args.before/name/"depth_valid_mask.png")) > 0
    amask = np.asarray(Image.open(args.after/name/"depth_valid_mask.png")) > 0
    both = bmask & amask
    error = np.abs(before[both] - after[both])
    b_rgb = np.asarray(Image.open(args.before/name/"rgb.png").convert("RGB"), dtype=np.int16)
    a_rgb = np.asarray(Image.open(args.after/name/"rgb.png").convert("RGB"), dtype=np.int16)
    b_points = np.load(args.before/name/"points_world.npy")
    a_points = np.load(args.after/name/"points_world.npy")
    report.append({"view": name, "before_valid_fraction": float(bmask.mean()),
                   "after_valid_fraction": float(amask.mean()),
                   "both_valid_mean_absolute_depth_difference_m": float(error.mean()) if len(error) else None,
                   "rgb_mean_absolute_difference": float(np.abs(b_rgb-a_rgb).mean()),
                   "pointcloud_max_absolute_difference_m": float(np.abs(b_points-a_points).max())
                        if b_points.shape == a_points.shape and b_points.size else None,
                   "new_valid_pixels": int((amask & ~bmask).sum()),
                   "lost_valid_pixels": int((bmask & ~amask).sum())})
    top = i*(480*rows_per_view+35)
    draw.text((5, top+5), f"{name} | source only (left) / repair layer (right)", fill="black")
    for col, root in enumerate((args.before, args.after)):
        canvas.paste(Image.open(root/name/"rgb.png").convert("RGB"), (col*640, top+35))
        canvas.paste(Image.open(root/name/"depth_preview.png").convert("RGB"), (col*640, top+35+480))
        if args.include_mask:
            canvas.paste(Image.open(root/name/"depth_valid_mask.png").convert("RGB"),
                         (col*640, top+35+960))
args.output.parent.mkdir(parents=True, exist_ok=True)
canvas.save(args.output)
report_path = args.output.parent / (f"{args.output.stem}_comparison.json" if args.view else
                                    "view_comparison.json")
report_path.write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(report, indent=2))
