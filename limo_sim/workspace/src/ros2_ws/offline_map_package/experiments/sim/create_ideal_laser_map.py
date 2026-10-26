#!/usr/bin/env python3
"""Export the Gazebo circuit wall collisions as a raw laser occupancy map.

Uses the circuit map's resolution and world axes. Wall cells are occupied,
the enclosed arena is free and the exterior is unknown. No inflation is applied.
Only horizontal box collisions and ordinary planar SDF poses are supported.
"""

import argparse
from collections import deque
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


SOURCE = Path(__file__).resolve().parents[4]
IDEAL_DIRECTORY = Path(__file__).resolve().parent / 'ideal_map'
DEFAULT_WORLD = SOURCE / 'custom_start/worlds/limo_circuit_world.world'
DEFAULT_MODEL = SOURCE / 'custom_start/models/walls/model.sdf'


def planar_pose(element):
    """Read XY/yaw while rejecting named frames and tilted geometry."""
    pose = element.find('pose')
    if pose is None:
        return 0.0, 0.0, 0.0
    if pose.get('relative_to') or pose.get('frame'):
        raise ValueError('Named SDF pose frames are not supported')
    values = [float(value) for value in (pose.text or '').split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('Expected a finite six-component SDF pose')
    if abs(values[3]) > 1e-9 or abs(values[4]) > 1e-9:
        raise ValueError('Wall collisions must be horizontal boxes')
    return values[0], values[1], values[5]


def compose(parent, child):
    """Compose planar poses in world coordinates."""
    x, y, yaw = parent
    dx, dy, angle = child
    return (x + math.cos(yaw) * dx - math.sin(yaw) * dy,
            y + math.sin(yaw) * dx + math.cos(yaw) * dy, yaw + angle)


def load_wall_boxes(model_path, world_path):
    """Read actual collision dimensions and all enclosing SDF transforms."""
    model = ET.parse(model_path).getroot().find('model')
    world = ET.parse(world_path).getroot().find('world')
    if model is None or world is None:
        raise ValueError('Expected a Gazebo model and world')
    uri = 'model://' + model.get('name')
    include = next((item for item in world.findall('include')
                    if item.findtext('uri', '').rstrip('/') == uri), None)
    if include is None:
        raise ValueError('Wall model is not included in the selected world')
    # SDF include/pose overrides model/pose rather than composing with it.
    # https://github.com/gazebosim/sdf_tutorials/blob/master/docs/spec_world/tutorial1.md
    placement = planar_pose(include if include.find('pose') is not None else model)
    boxes = []
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            dimensions = [float(value) for value in
                          collision.findtext('geometry/box/size', '').split()]
            if len(dimensions) != 3 or not all(
                    math.isfinite(value) and value > 0 for value in dimensions):
                raise ValueError('Expected positive wall collision box dimensions')
            pose = compose(compose(placement, planar_pose(link)), planar_pose(collision))
            boxes.append((pose, dimensions[:2]))
    if not boxes:
        raise ValueError('No wall collisions found')
    return boxes


def rasterize(boxes, resolution):
    """Sample wall footprints at cell centers, then mask the exterior by flood fill."""
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Resolution must be finite and positive')
    corners = [compose(pose, (sx * width / 2, sy * height / 2, 0))[:2]
               for pose, (width, height) in boxes for sx in (-1, 1) for sy in (-1, 1)]
    low = [math.floor(min(point[i] for point in corners) / resolution) - 1
           for i in (0, 1)]
    high = [math.ceil(max(point[i] for point in corners) / resolution) + 1
            for i in (0, 1)]
    width, height = (high[i] - low[i] for i in (0, 1))
    if width * height > 4000000:
        raise ValueError('Laser map exceeds 4000000 cells')
    ox, oy = (value * resolution for value in low)
    rows = [bytearray(width) for _ in range(height)]
    for pose, (size_x, size_y) in boxes:
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        for row in range(height):
            dy = oy + (row + 0.5) * resolution - py
            for column in range(width):
                dx = ox + (column + 0.5) * resolution - px
                if abs(c * dx + s * dy) <= size_x / 2 and abs(-s * dx + c * dy) <= size_y / 2:
                    rows[row][column] = 100
    pending = deque([(0, 0)])
    rows[0][0] = 255
    while pending:
        x, y = pending.popleft()
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= nx < width and 0 <= ny < height and rows[ny][nx] == 0:
                rows[ny][nx] = 255
                pending.append((nx, ny))
    if not any(0 in row for row in rows):
        raise ValueError('Wall raster does not enclose a free arena at this resolution')
    return {'mode': 'raw', 'resolution': resolution, 'origin': [ox, oy, 0.0],
            'negate': 0, 'occupied_thresh': 1.0, 'free_thresh': 0.0}, rows


def save_map(output_stem, metadata, rows):
    """Write Nav2 raw PGM/YAML; image rows run from world top to bottom."""
    pgm = Path(str(output_stem) + '.pgm')
    pgm.parent.mkdir(parents=True, exist_ok=True)
    header = 'P5\n# Gazebo wall collisions\n{} {}\n255\n'.format(len(rows[0]), len(rows))
    pgm.write_bytes(header.encode('ascii') + b''.join(reversed(rows)))
    metadata = {'image': pgm.name, **metadata}
    Path(str(output_stem) + '.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gazebo-world', type=Path, default=DEFAULT_WORLD)
    parser.add_argument('--gazebo-model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--reference-map', type=Path, default=IDEAL_DIRECTORY / 'ideal_map.yaml')
    parser.add_argument('--output', type=Path, default=IDEAL_DIRECTORY / 'ideal_laser_map')
    args = parser.parse_args()
    reference = yaml.safe_load(args.reference_map.read_text(encoding='utf-8'))
    boxes = load_wall_boxes(args.gazebo_model, args.gazebo_world)
    metadata, rows = rasterize(boxes, float(reference['resolution']))
    save_map(args.output, metadata, rows)
    print('Saved {}.pgm/yaml: {} x {} cells, resolution {} m, origin {}'.format(
        args.output, len(rows[0]), len(rows), metadata['resolution'], metadata['origin']))


if __name__ == '__main__':
    main()
