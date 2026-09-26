#!/usr/bin/env python3
"""Plot robot trajectories from capture_auto_rgbd manifest files."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def load_map(path: Path):
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    image = Path(str(cfg["image"]))
    if not image.is_absolute():
        image = path.parent / image
    pixels = np.asarray(Image.open(image).convert("L"), dtype=np.uint8)
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    resolution = float(cfg["resolution"])
    return np.flipud(pixels), float(origin[0]), float(origin[1]), resolution


def load_rows(manifest: Path):
    return [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines() if line.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="capture output root or one episode directory")
    parser.add_argument("--map", dest="map_yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="PNG path")
    args = parser.parse_args()

    root = args.input.expanduser().resolve()
    episodes = sorted(root.glob("episode_*")) if (root / "episode_000").is_dir() else [root]
    image, origin_x, origin_y, resolution = load_map(args.map_yaml.expanduser().resolve())
    height, width = image.shape
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
    ax.imshow(image, cmap="gray", origin="lower",
              extent=(origin_x, origin_x + width * resolution,
                      origin_y, origin_y + height * resolution),
              vmin=0, vmax=255)

    colors = plt.cm.viridis(np.linspace(0.1, 0.9, max(1, len(episodes))))
    quality_label_used = False
    for color, episode in zip(colors, episodes):
        manifest = episode / "left" / "manifest.jsonl"
        if not manifest.is_file():
            continue
        rows = load_rows(manifest)
        if not rows:
            continue
        poses = np.asarray([row["robot_pose_world"] for row in rows], dtype=float)
        label = episode.name
        ax.plot(poses[:, 0], poses[:, 1], color=color, linewidth=2, label=label)
        flagged = np.asarray([not row.get("frame_quality_accepted", True) for row in rows], dtype=bool)
        if np.any(flagged):
            ax.scatter(poses[flagged, 0], poses[flagged, 1], color="darkorange",
                       marker="x", s=16, zorder=5,
                       label="quality flagged" if not quality_label_used else None)
            quality_label_used = True
        ax.scatter(poses[0, 0], poses[0, 1], color=color, marker="o", s=35, edgecolors="black", zorder=4)
        ax.scatter(poses[-1, 0], poses[-1, 1], color=color, marker="X", s=55, edgecolors="black", zorder=4)
        targets = [row.get("target_world") for row in rows if row.get("target_world") is not None]
        if targets:
            target = np.asarray(targets[-1], dtype=float)
            ax.scatter(target[0], target[1], color=color, marker="*", s=100, edgecolors="black", zorder=4)

    ax.set_title("Galbot S1 trajectory")
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    if ax.get_legend_handles_labels()[0]:
        ax.legend()
    args.output.expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output.expanduser().resolve(), bbox_inches="tight")
    plt.close(fig)
    print(args.output.expanduser().resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
