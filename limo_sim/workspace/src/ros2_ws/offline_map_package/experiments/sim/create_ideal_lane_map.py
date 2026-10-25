#!/usr/bin/env python3
"""Build ideal lane and boardwalk class maps from the Gazebo circuit texture.

Run with Python 3, Pillow, NumPy, SciPy and PyYAML installed. Pixels are assigned to the
nearest reference color in the original texture, without dithering. Both
road gray and its dark outline become blue; yellow markings become cyan;
the light background and white markings become white. This also assigns
antialiased edge pixels to a single class.

A second step mirrors visual_ptcld's boardwalk classification using two
Euclidean distance transforms instead of point neighbor queries. Boardwalk
and interior boardwalk become magenta; other white markings stay white.
The distances and propagation are metric and nonrecursive.

The last step reads terrain dimensions/pose from Gazebo SDF and costs/resolution
from config/mapping_sim.yaml, then writes ideal_map.pgm and ideal_map.yaml in
Nav2 raw mode. Each cell takes the maximum cost of its source pixel centers,
preserving thin markings. Grid bounds are rounded out to complete cells;
unsampled cells remain unknown. Origin is expressed in Gazebo world coordinates.

The two color images keep the original dimensions/orientation. The final map
uses the top-face UV orientation of Gazebo's box mesh and a metric grid.
"""

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


PACKAGE_DIRECTORY = Path(__file__).resolve().parents[2]
SOURCE_DIRECTORY = Path(__file__).resolve().parents[4]
# Support direct execution from the source tree without sourcing ROS or installing it.
sys.path.insert(0, str(PACKAGE_DIRECTORY))
from offline_map_package.map_io import map_pgm, map_yaml  # noqa: E402
from offline_map_package.semantic_grid import Geometry  # noqa: E402


IDEAL_MAP_DIRECTORY = Path(__file__).resolve().parent / 'ideal_map'
DEFAULT_MODEL_SDF = SOURCE_DIRECTORY / 'custom_start/models/plane_test/model.sdf'
DEFAULT_WORLD_SDF = SOURCE_DIRECTORY / 'custom_start/worlds/limo_circuit_world.world'
DEFAULT_MAPPING_CONFIG = PACKAGE_DIRECTORY / 'config/mapping_sim.yaml'
SOURCE_COLORS = (
    (102, 102, 102),  # Road surface.
    (32, 32, 32),    # Dark road outline.
    (255, 203, 0),   # Yellow markings.
    (242, 242, 242),  # Background and white markings.
)
OUTPUT_COLORS = (
    (0, 0, 255),     # Road: blue.
    (0, 255, 255),   # Yellow markings: cyan/turquoise.
    (255, 255, 255),  # Everything else: white.
)
CLASS_COLORS = OUTPUT_COLORS + ((255, 0, 255),)  # Boardwalk: magenta.
BLUE_RADIUS_MIN_M = 0.07
BLUE_RADIUS_MAX_M = 0.25
BOARDWALK_PROPAGATION_RADIUS_M = 0.07


def image_palette(colors):
    """Expand class RGB colors into a full indexed-image palette."""
    colors = colors + (colors[-1],) * (256 - len(colors))
    return [channel for color in colors for channel in color]


def create_ideal_lane_map(input_path, output_path):
    """Write an indexed PNG with exactly the three requested class colors."""
    input_path, output_path = Path(input_path), Path(output_path)
    if input_path.resolve() == output_path.resolve():
        raise ValueError('Input and output paths must differ')

    reference = Image.new('P', (1, 1))
    reference_colors = SOURCE_COLORS + (SOURCE_COLORS[-1],) * (256 - len(SOURCE_COLORS))
    reference.putpalette([channel for color in reference_colors for channel in color])
    output_palette = image_palette(OUTPUT_COLORS)
    class_lookup = [0, 0, 1] + [2] * 253

    with Image.open(input_path) as source:
        output = Image.new('P', source.size, 2)
        output.putpalette(output_palette)
        # Process strips to avoid multiple full-size copies of the large texture.
        for top in range(0, source.height, 256):
            box = (0, top, source.width, min(top + 256, source.height))
            rgba = source.crop(box).convert('RGBA')
            white = Image.new('RGBA', rgba.size, (255, 255, 255, 255))
            rgb = Image.alpha_composite(white, rgba).convert('RGB')
            classified = rgb.quantize(palette=reference, dither=0)
            classified = classified.point(class_lookup)
            classified.putpalette(output_palette)
            output.paste(classified, (0, top))

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output.save(output_path, format='PNG')
        return output.size, output.histogram()[:3]


def classify_boardwalk_pixels(labels, pixel_size_m, minimum, maximum, propagation):
    """Mirror boardwalk.py's two passes on a dense, metric pixel-center grid.

    Labels: 0 road, 1 yellow markings, 2 other white, 3 boardwalk. As in the
    point classifier, missing road sources leave the white labels unchanged.
    Interior boardwalk (> maximum) and ordinary boardwalk share output label 3.
    """
    classified = labels.copy()
    road = labels == 0
    white = labels == 2
    if not road.any() or not white.any():
        return classified

    distance_blue = distance_transform_edt(~road, sampling=pixel_size_m)
    eligible = white & (distance_blue <= maximum)
    interior = white & (distance_blue > maximum)
    seeds = eligible & (distance_blue > minimum)
    selected = seeds | interior
    del distance_blue

    if seeds.any() and propagation > 0:
        distance_seed = distance_transform_edt(~seeds, sampling=pixel_size_m)
        # Only initial seeds propagate, never newly classified pixels.
        selected |= eligible & (distance_seed < propagation)
    classified[selected] = 3
    return classified


def create_ideal_class_map(lane_path, output_path, width_m, height_m,
                          minimum, maximum, propagation):
    """Classify strips with enough halo for both distance passes, without resizing."""
    lane_path, output_path = Path(lane_path), Path(output_path)
    if lane_path.resolve() == output_path.resolve():
        raise ValueError('Lane and class map paths must differ')
    if (not all(math.isfinite(value) for value in
                (width_m, height_m, minimum, maximum, propagation))
            or width_m <= 0 or height_m <= 0
            or not 0 <= minimum <= maximum or propagation < 0):
        raise ValueError('Invalid physical size or boardwalk distance thresholds')

    with Image.open(lane_path) as source:
        if source.mode != 'P' or source.getpalette()[:9] != image_palette(OUTPUT_COLORS)[:9]:
            raise ValueError('Expected the indexed lane image generated by step 1')
        if source.histogram()[0] == 0:
            raise ValueError('Cannot classify boardwalk without road pixels')
        pixel_size_m = (height_m / source.height, width_m / source.width)
        # A propagated pixel needs seeds within propagation, whose road sources
        # may be maximum farther away. Extra rows protect strict boundaries.
        halo = math.ceil((maximum + propagation) / pixel_size_m[0]) + 2
        output = Image.new('P', source.size, 2)
        output.putpalette(image_palette(CLASS_COLORS))
        for top in range(0, source.height, 256):
            bottom = min(top + 256, source.height)
            extended_top = max(0, top - halo)
            extended_bottom = min(source.height, bottom + halo)
            labels = np.asarray(source.crop(
                (0, extended_top, source.width, extended_bottom)))
            if np.any(labels > 2):
                raise ValueError('Unexpected label in the lane image')
            if np.any(labels == 0):
                classified = classify_boardwalk_pixels(
                    labels, pixel_size_m, minimum, maximum, propagation)
            else:
                # All road sources are farther than the complete halo: white
                # pixels in the core are interior boardwalk. A globally absent
                # road is rejected before processing the strips.
                classified = labels.copy()
                classified[labels == 2] = 3
            core = classified[top - extended_top:bottom - extended_top]
            strip = Image.fromarray(core)
            strip.putpalette(image_palette(CLASS_COLORS))
            output.paste(strip, (0, top))

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output.save(output_path, format='PNG')
        return output.size, output.histogram()[:4], pixel_size_m


def planar_sdf_pose(element):
    """Read an ordinary planar SDF pose; reject unsupported frame references."""
    pose = element.find('pose')
    if pose is None:
        return (0.0, 0.0, 0.0)
    if pose.get('relative_to') or pose.get('frame'):
        raise ValueError('Named SDF pose frames are not supported by this exporter')
    values = [float(value) for value in (pose.text or '').split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('Expected a finite six-component SDF pose')
    if abs(values[3]) > 1e-9 or abs(values[4]) > 1e-9:
        raise ValueError('The circuit surface must be horizontal')
    return values[0], values[1], values[5]


def compose_pose(parent, child):
    """Compose planar translations and yaw angles."""
    x, y, yaw = parent
    dx, dy, angle = child
    return (x + math.cos(yaw) * dx - math.sin(yaw) * dy,
            y + math.sin(yaw) * dx + math.cos(yaw) * dy, yaw + angle)


def load_gazebo_plane(model_path, world_path):
    """Return box XY dimensions and world pose of the circuit's textured surface."""
    model = ET.parse(model_path).getroot().find('model')
    if model is None:
        raise ValueError('Gazebo model file does not contain a model')
    name = model.get('name')
    world = ET.parse(world_path).getroot().find('world')
    if world is None:
        raise ValueError('Gazebo world file does not contain a world')
    inline = next((item for item in world.findall('model')
                   if item.get('name') == name), None)
    if inline is not None:
        model, placement = inline, (0.0, 0.0, 0.0)
    else:
        include = next((item for item in world.findall('include')
                        if item.findtext('uri', '').rstrip('/') == 'model://' + name), None)
        if include is None:
            raise ValueError('Circuit model not found in the selected Gazebo world')
        placement = planar_sdf_pose(include)
    placement = compose_pose(placement, planar_sdf_pose(model))
    for link in model.findall('link'):
        for visual in link.findall('visual'):
            if visual.findtext('material/script/name') != 'vrc/circuit':
                continue
            size = visual.findtext('geometry/box/size')
            if size is None:
                raise ValueError('Circuit visual must use box geometry')
            dimensions = tuple(float(value) for value in size.split())
            if (len(dimensions) != 3
                    or not all(math.isfinite(value) and value > 0 for value in dimensions)):
                raise ValueError('Invalid circuit box dimensions')
            pose = compose_pose(compose_pose(placement, planar_sdf_pose(link)),
                                planar_sdf_pose(visual))
            return dimensions[:2], pose
    raise ValueError('Circuit material vrc/circuit not found in the Gazebo model')


def load_mapping_settings(path):
    """Read the actual offline mapping profile, without duplicating its costs."""
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    settings = config['semantic_mapper']
    resolution = float(settings['resolution'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid mapping resolution')
    costs = [settings[name + '_cost'] for name in ('yellow', 'white', 'boardwalk')]
    if any(isinstance(value, bool) or not isinstance(value, int)
           or not 1 <= value <= 100 for value in costs):
        raise ValueError('Semantic costs must be integers between 1 and 100')
    max_cells = int(settings.get('max_output_cells', 4000000))
    if max_cells < 1:
        raise ValueError('Invalid map output cell limit')
    return resolution, np.array([0] + costs, dtype=np.int8), max_cells


def rasterize_cost_map(class_path, size_xy, pose, resolution, costs, max_cells):
    """Rasterize class pixel centers with the top-face UVs of Gazebo's unit_box.

    Gazebo MeshManager::CreateBox: top-left texture corner is (+X, +Y),
    texture rows increase toward -X and columns toward -Y. Grid array rows
    increase toward local +Y; columns toward +X. A max reduction prevents
    thin yellow/white lines from disappearing among road pixels.

    UV reference: github.com/gazebosim/gazebo/blob/gazebo11/gazebo/common/MeshManager.cc
    """
    size_xy = np.asarray(size_xy, dtype=float)
    if (size_xy.shape != (2,) or not np.isfinite(size_xy).all()
            or np.any(size_xy <= 0) or not math.isfinite(resolution) or resolution <= 0
            or len(pose) != 3 or not np.isfinite(pose).all()):
        raise ValueError('Invalid plane size, pose or map resolution')
    if len(costs) != 4 or np.any(np.asarray(costs) < 0) or np.any(np.asarray(costs) > 100):
        raise ValueError('Expected four costs in color-label order')
    low = np.floor(-0.5 * size_xy / resolution).astype(np.int64)
    high = np.ceil(0.5 * size_xy / resolution).astype(np.int64)
    width, height = (int(value) for value in high - low)
    if width * height > max_cells:
        raise MemoryError('Ideal map exceeds max_output_cells from the mapping profile')
    origin = compose_pose(pose, (low[0] * resolution, low[1] * resolution, 0.0))
    geometry = Geometry(resolution, width, height, origin)
    values = np.full((height, width), -1, dtype=np.int8)

    with Image.open(class_path) as source:
        if source.mode != 'P' or source.getpalette()[:12] != image_palette(CLASS_COLORS)[:12]:
            raise ValueError('Expected the indexed class image generated by step 2')
        y = size_xy[1] * (0.5 - (np.arange(source.width) + 0.5) / source.width)
        rows = np.floor(y / resolution).astype(np.int64) - low[1]
        for top in range(0, source.height, 256):
            bottom = min(source.height, top + 256)
            labels = np.asarray(source.crop((0, top, source.width, bottom)))
            if np.any(labels > 3):
                raise ValueError('Unexpected color label in the class image')
            x = size_xy[0] * (0.5 - (np.arange(top, bottom) + 0.5) / source.height)
            columns = np.floor(x / resolution).astype(np.int64) - low[0]
            np.maximum.at(values, (rows[None, :], columns[:, None]), costs[labels])
    return geometry, values


def save_ideal_map(class_path, output_stem, size_xy, pose, resolution, costs, max_cells):
    """Write the same raw PGM encoding and YAML metadata as semantic_mapper."""
    geometry, values = rasterize_cost_map(
        class_path, size_xy, pose, resolution, costs, max_cells)
    pgm_path = Path(str(output_stem) + '.pgm')
    yaml_path = Path(str(output_stem) + '.yaml')
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    pgm_path.write_bytes(map_pgm(values, 'raw'))
    yaml_path.write_text(map_yaml(pgm_path.name, geometry, 'raw'), encoding='utf-8')
    return geometry, values, pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path,
                        default=IDEAL_MAP_DIRECTORY / 'circuit.png')
    parser.add_argument('--output', type=Path,
                        default=IDEAL_MAP_DIRECTORY / 'ideal_map_lane_detection.png')
    parser.add_argument('--class-output', type=Path,
                        default=IDEAL_MAP_DIRECTORY / 'ideal_class_map.png')
    parser.add_argument('--width-m', type=float,
                        help='Override texture width in meters (Gazebo local Y)')
    parser.add_argument('--height-m', type=float,
                        help='Override texture height in meters (Gazebo local X)')
    parser.add_argument('--gazebo-model', type=Path, default=DEFAULT_MODEL_SDF)
    parser.add_argument('--gazebo-world', type=Path, default=DEFAULT_WORLD_SDF)
    parser.add_argument('--mapping-config', type=Path, default=DEFAULT_MAPPING_CONFIG)
    parser.add_argument('--map-output', type=Path, default=IDEAL_MAP_DIRECTORY / 'ideal_map',
                        help='Output filename stem for the final PGM/YAML pair')
    parser.add_argument('--blue-radius-min-m', type=float, default=BLUE_RADIUS_MIN_M)
    parser.add_argument('--blue-radius-max-m', type=float, default=BLUE_RADIUS_MAX_M)
    parser.add_argument('--boardwalk-propagation-radius-m', type=float,
                        default=BOARDWALK_PROPAGATION_RADIUS_M)
    args = parser.parse_args()
    paths = (args.input, args.output, args.class_output,
             Path(str(args.map_output) + '.pgm'), Path(str(args.map_output) + '.yaml'))
    if len({path.resolve() for path in paths}) != len(paths):
        parser.error('Input and all output files must have different paths')
    plane_size, plane_pose = load_gazebo_plane(args.gazebo_model, args.gazebo_world)
    width_m = args.width_m if args.width_m is not None else plane_size[1]
    height_m = args.height_m if args.height_m is not None else plane_size[0]
    resolution, costs, max_cells = load_mapping_settings(args.mapping_config)
    size, counts = create_ideal_lane_map(args.input, args.output)
    print('Saved: {}'.format(args.output))
    print('Dimensions: {} x {} pixels'.format(*size))
    print('Pixels: road={}, yellow markings={}, other={}'.format(*counts))
    print('Step 2: metric boardwalk distance transforms...', flush=True)
    size, counts, pixel_size = create_ideal_class_map(
        args.output, args.class_output, width_m, height_m,
        args.blue_radius_min_m, args.blue_radius_max_m,
        args.boardwalk_propagation_radius_m)
    print('Saved: {}'.format(args.class_output))
    print('Dimensions: {} x {} pixels'.format(*size))
    print('Meters per pixel: rows={:.9f}, columns={:.9f}'.format(*pixel_size))
    print('Pixels: road={}, yellow markings={}, white={}, boardwalk={}'.format(*counts))
    print('Step 3: metric raw cost map...', flush=True)
    geometry, values, pgm_path, yaml_path = save_ideal_map(
        args.class_output, args.map_output, (height_m, width_m), plane_pose,
        resolution, costs, max_cells)
    print('Saved: {} and {}'.format(pgm_path, yaml_path))
    print('Gazebo circuit: X={:.6f} m, Y={:.6f} m'.format(height_m, width_m))
    print('Map: {} x {} cells at {} m/cell; origin={}'.format(
        geometry.width, geometry.height, geometry.resolution, geometry.origin))
    print('Costs: road={}, yellow={}, white={}, boardwalk={}'.format(*costs))
    print('Cost cell counts: {}'.format(dict(zip(*np.unique(values, return_counts=True)))))


if __name__ == '__main__':
    main()
