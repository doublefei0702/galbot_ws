"""Safety checks for the opt-in local area and rotated map coordinates."""
from dataclasses import replace
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "exts/galbot.mobility_gen/galbot/mobility_gen"))
sys.path.insert(0, str(ROOT / "scripts"))

from local_workspace import load_workspace  # noqa: E402
from build_local_warehouse import _grid_xy, _world_xy  # noqa: E402
from capture_auto_rgbd import EpisodeWriter, _camera_attitude  # noqa: E402
from auto_navigation import astar, load_ros_grid, sample_reachable_goal  # noqa: E402


def test_footprint_and_swept_exclusion():
    workspace = load_workspace(ROOT / "configs/scene_with_collision_local.yaml")
    assert workspace.point_reason(9, 0) is None
    assert workspace.point_reason(11.6, 0) == "workspace_boundary"
    assert workspace.segment_reason((9, 0), (13, 0)) == "workspace_boundary"
    with_hole = replace(workspace, excludes=(((9.5, -0.5), (10.5, -0.5),
                                              (10.5, 0.5), (9.5, 0.5)),))
    assert with_hole.point_reason(8, 0) is None
    assert with_hole.point_reason(11.25, 0) is None
    assert with_hole.segment_reason((8, 0), (11.25, 0)) == "excluded_region"


def test_rotated_map_origin_roundtrip():
    origin = (3.0, -2.0, math.pi / 2)
    world = _world_xy(12, 7, origin, 0.05)
    col, row = _grid_xy(*world, origin, 0.05)
    assert abs(col - 12.5) < 1e-9
    assert abs(row - 7.5) < 1e-9


def test_raw_depth_and_mask_keep_no_return_distinct(tmp_path):
    import json
    import numpy as np
    from PIL import Image

    workspace = load_workspace(ROOT / "configs/scene_with_collision_local.yaml")
    writer = EpisodeWriter(tmp_path, 0, {"left": {}}, workspace)
    depth = np.array([[np.inf, np.nan, 0.0], [0.05, 1.0, 26.0]], dtype=np.float32)
    position = np.array([9.0, 0.0, workspace.ground_z_m + 0.60])
    writer.write(0, {"left": (np.zeros((2, 3, 3), dtype=np.uint8), depth,
                               position, np.array([1.0, 0.0, 0.0, 0.0]), np.eye(4))}, {})
    high_position = position.copy()
    high_position[2] += 0.3
    writer.write(1, {"left": (np.zeros((2, 3, 3), dtype=np.uint8), depth,
                               high_position, np.array([1.0, 0.0, 0.0, 0.0]), np.eye(4))},
                 {"frame_quality_accepted": False,
                  "frame_quality_failures": ["camera_height_outside_tolerance"]})
    writer.close()

    folder = tmp_path / "episode_000" / "left"
    raw = np.load(folder / "depth_raw/000000.npy")
    mask = np.asarray(Image.open(folder / "depth_valid_mask/000000.png"))
    millimetres = np.asarray(Image.open(folder / "depth/000000.png"))
    rows = [json.loads(line) for line in (folder / "manifest.jsonl").read_text().splitlines()]
    row = rows[0]
    assert np.array_equal(raw, depth, equal_nan=True)
    assert np.array_equal(mask, [[0, 0, 0], [0, 255, 0]])
    assert np.array_equal(millimetres, [[0, 0, 0], [0, 1000, 0]])
    assert row["invalid_depth_pixels"] == 3
    assert row["out_of_range_depth_pixels"] == 2
    assert row["camera_below_height_reference"] is False
    assert row["camera_outside_height_tolerance"] is False
    assert rows[1]["camera_outside_height_tolerance"] is True
    assert rows[1]["frame_quality_accepted"] is False
    assert rows[1]["frame_quality_failures"] == ["camera_height_outside_tolerance"]
    assert (folder / "pose.txt").read_text().count("# frame ") == 2


def test_camera_attitude_rejects_arm_rotation_even_with_good_depth():
    import numpy as np

    workspace = replace(load_workspace(ROOT / "configs/scene_with_collision_expanded.yaml"),
                        camera_pitch_deg=-10.0, camera_yaw_deg=0.0,
                        camera_pitch_tolerance_deg=10.0, camera_yaw_tolerance_deg=10.0)
    matrix = np.eye(4)
    matrix[:3, 2] = [math.cos(math.radians(10)), 0.0, -math.sin(math.radians(10))]
    assert not _camera_attitude(matrix, 0.0, workspace)["camera_orientation_outside_tolerance"]
    matrix[:3, 2] = [-0.34, -0.08, 0.937]
    assert _camera_attitude(matrix, 0.0, workspace)["camera_orientation_outside_tolerance"]


def test_expanded_candidate_keeps_obstacle_detour_and_blocks_exterior():
    workspace = load_workspace(ROOT / "configs/scene_with_collision_expanded.yaml")
    grid = load_ros_grid("/root/gpufree-data/data_usd/scene_local_expanded/local_map.yaml")
    safe = grid.inflated(workspace.clearance_m)
    route = grid.inflated(workspace.clearance_m + 0.20)
    start, goal = (23.0, -7.0), (28.0, -7.0)
    assert not safe.segment_is_free(start, goal)
    path = astar(route, start, goal)
    assert path is not None and route.path_is_free(path)
    assert max(y for _, y in path) > -5.0  # route goes around the north side
    assert workspace.point_reason(30.0, -7.0) == "workspace_boundary"
    assert not safe.is_free_world(30.0, -7.0)


def test_sampled_paths_obey_full_workspace_clearance():
    import numpy as np

    workspace = load_workspace(ROOT / "configs/scene_with_collision_expanded.yaml")
    grid = load_ros_grid("/root/gpufree-data/data_usd/scene_local_expanded/local_map.yaml")
    safe = workspace.mask_grid_for_footprint(grid.inflated(workspace.clearance_m))
    rng = np.random.default_rng(7)
    for _ in range(8):
        result = sample_reachable_goal(
            grid, safe, (9.0, 0.0), rng, 1.5, 64,
            path_validator=lambda path: all(
                workspace.segment_reason(a, b, safe) is None for a, b in zip(path, path[1:])))
        assert result is not None
        assert all(workspace.segment_reason(a, b, safe) is None
                   for a, b in zip(result[1], result[1][1:]))


def test_current_ros_navigator_uses_the_expanded_keepout_map():
    import pytest

    sys.path.insert(0, "/root/gpufree-data/navigation/src/galbot_navigation")
    from galbot_navigation.grid_map import OccupancyGrid2D
    from galbot_navigation.planner import AStarPlanner

    map_yaml = "/root/gpufree-data/data_usd/scene_local_expanded/local_map.yaml"
    grid = OccupancyGrid2D.from_ros_yaml(map_yaml).inflated(0.70)
    planner = AStarPlanner(grid)
    assert not grid.is_segment_free(grid.world_to_grid(23, -7), grid.world_to_grid(28, -7))
    assert planner.plan((23, -7), (28, -7)) is not None
    assert planner.plan((9, 0), (28, -7)) is not None
    with pytest.raises(ValueError, match="goal is not in free space"):
        planner.plan((9, 0), (30, -7))
