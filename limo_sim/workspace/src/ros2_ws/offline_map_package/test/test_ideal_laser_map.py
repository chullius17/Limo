"""Check Gazebo placement, metric wall faces and the shipped laser map."""

import importlib.util
import math
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'experiments/sim/create_ideal_laser_map.py'
spec = importlib.util.spec_from_file_location('ideal_laser_export', SCRIPT)
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)


def arena():
    boxes = export.load_wall_boxes(export.DEFAULT_MODEL, export.DEFAULT_WORLD)
    return export.rasterize(boxes, 0.05)


def cell(metadata, rows, x, y):
    ox, oy, _ = metadata['origin']
    return rows[math.floor((y - oy) / metadata['resolution'])][
        math.floor((x - ox) / metadata['resolution'])]


def test_world_include_pose_overrides_model_offset():
    boxes = export.load_wall_boxes(export.DEFAULT_MODEL, export.DEFAULT_WORLD)
    # model.sdf contains an offset, but the world explicitly places walls at zero.
    assert len(boxes) == 4
    assert boxes[0][0][:2] == pytest.approx((-6.0984, 0.0))
    assert boxes[1][0][:2] == pytest.approx((0.0, -4.0338375))
    assert boxes[2][0][:2] == pytest.approx((6.0984, 0.0))
    assert boxes[3][0][:2] == pytest.approx((0.0, 4.0338375))


def test_wall_faces_free_interior_and_unknown_exterior():
    metadata, rows = arena()
    assert cell(metadata, rows, 0, 0) == 0
    # Inner edges of occupied cells: tolerance is half a 5 cm cell.
    occupied_x = [metadata['origin'][0] + (i + 0.5) * 0.05
                  for i in range(len(rows[0]))
                  if cell(metadata, rows, metadata['origin'][0] + (i + 0.5) * 0.05, 0) == 100]
    occupied_y = [metadata['origin'][1] + (i + 0.5) * 0.05
                  for i in range(len(rows))
                  if cell(metadata, rows, 0, metadata['origin'][1] + (i + 0.5) * 0.05) == 100]
    inner_x = 6.0984 - 0.190575 / 2
    inner_y = 4.0338375 - 0.190575 / 2
    assert min(x for x in occupied_x if x > 0) - 0.025 == pytest.approx(inner_x, abs=0.025)
    assert max(x for x in occupied_x if x < 0) + 0.025 == pytest.approx(-inner_x, abs=0.025)
    assert min(y for y in occupied_y if y > 0) - 0.025 == pytest.approx(inner_y, abs=0.025)
    assert max(y for y in occupied_y if y < 0) + 0.025 == pytest.approx(-inner_y, abs=0.025)
    for x, y in ((-6.225, 0), (6.225, 0), (0, -4.175), (0, 4.175)):
        assert cell(metadata, rows, x, y) == 255


def test_shipped_map_matches_gazebo_and_pgm_orientation(tmp_path):
    metadata, rows = arena()
    stem = tmp_path / 'laser'
    export.save_map(stem, metadata, rows)
    shipped = export.IDEAL_DIRECTORY / 'ideal_laser_map'
    assert stem.with_suffix('.pgm').read_bytes() == shipped.with_suffix('.pgm').read_bytes()
    actual = yaml.safe_load(shipped.with_suffix('.yaml').read_text())
    expected = yaml.safe_load(stem.with_suffix('.yaml').read_text())
    assert actual == {**expected, 'image': 'ideal_laser_map.pgm'}
    pixels = stem.with_suffix('.pgm').read_bytes().split(b'\n', 4)[4]
    assert pixels[:len(rows[0])] == rows[-1]
    assert pixels[-len(rows[0]):] == rows[0]


def test_open_walls_are_rejected():
    with pytest.raises(ValueError, match='does not enclose'):
        export.rasterize([((0, 0, 0), (2, 0.2))], 0.05)
