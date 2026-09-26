#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import sys
import tempfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "exts/galbot.mobility_gen/galbot/mobility_gen/auto_navigation.py"
spec = importlib.util.spec_from_file_location("auto_navigation", MODULE)
planner = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = planner
spec.loader.exec_module(planner)


def test_inflation_blocks_narrow_gap_and_astar_uses_open_route():
    free = np.ones((15, 20), dtype=bool)
    free[7, 6:14] = False
    free[7, 8] = True  # narrow opening; 0.15 m footprint cannot pass it
    grid = planner.GridMap(free, 0.1, 0.0, 0.0, 0.0)
    safe = grid.inflated(0.15)
    assert not safe.is_free_cell(8, 7)
    path = planner.astar(safe, (0.25, 0.25), (1.75, 1.25))
    assert path is not None
    assert grid.path_is_free(path)


def test_segment_check_rejects_thin_wall_crossing():
    free = np.ones((10, 10), dtype=bool)
    free[5, 4:6] = False
    grid = planner.GridMap(free, 0.1, 0.0, 0.0, 0.0)
    assert not grid.segment_is_free((0.45, 0.45), (0.55, 0.95))


def test_ros_map_axis_conversion_round_trip(tmp_path):
    from PIL import Image

    image = tmp_path / "map.png"
    Image.fromarray(np.full((4, 5), 255, dtype=np.uint8)).save(image)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text("image: map.png\nresolution: 0.1\norigin: [1, 2, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    grid = planner.load_ros_grid(yaml_path)
    assert grid.is_free_world(1.05, 2.05)
    assert grid.world_to_grid(1.05, 2.05) == (0, 0)


if __name__ == "__main__":
    test_inflation_blocks_narrow_gap_and_astar_uses_open_route()
    test_segment_check_rejects_thin_wall_crossing()
    with tempfile.TemporaryDirectory() as directory:
        test_ros_map_axis_conversion_round_trip(Path(directory))
    print("auto navigation tests passed")
