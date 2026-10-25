#!/usr/bin/env python3
"""Reconstruct the latest per-cloud majority map from recorded cloud/TF inputs.

For each touched cell, the class with the most points in the current cloud
replaces its previous observation. Class counts are never accumulated over
time. Exact ties become unknown; untouched cells keep their last observation.
Road labels 1/5 and boardwalk labels 4/6 are merged as in SemanticGrid.

This deterministic offline reconstruction uses the complete recorded TF history
at each sensor timestamp and the last valid SLAM map as output geometry. It
does not simulate live queue drops or TF waiting deadlines. No log-odds,
evidence threshold, free-confirmation delay or median filter is applied.
"""

import argparse
from collections import Counter
import hashlib
import heapq
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
from PIL import Image
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException
import yaml


DIRECTORY = Path(__file__).resolve().parent
sys.path.insert(0, str(DIRECTORY.parents[1]))
from offline_map_package.map_io import map_pgm, map_yaml  # noqa: E402
from offline_map_package.map_filters import median_saved_semantic  # noqa: E402
from offline_map_package.semantic_grid import (  # noqa: E402
    CLASS_IDS, INPUT_CLASS_IDS, TILE_SIZE, Geometry, SemanticGrid,
    read_class_cloud, transform_xy,
)
from offline_map_package.semantic_mapper import (  # noqa: E402
    combine_semantic_and_laser, planar_pose, stamp_ns,
)


class LatestMajorityGrid(SemanticGrid):
    """Reuse the mapper's tiling/rendering, replacing only temporal integration."""

    def __init__(self, config):
        super().__init__(
            resolution=config['resolution'],
            costs=[config[name + '_cost'] for name in ('yellow', 'white', 'boardwalk')],
            # Every nonzero fraction is accepted; render still rejects exact ties.
            threshold=float(np.finfo(np.float32).eps),
            max_cells=config['max_cells'], max_output_cells=config['max_output_cells'])
        self.set_pose((0, 0), (0.0, 0.0, 0.0))
        self.tied_observations = 0
        self.cell_observations = 0

    def update(self, points, labels, key=(0, 0)):
        if key != (0, 0):
            raise ValueError('This baseline supports the recorded TF pose mode only')
        valid = np.isfinite(points).all(axis=1) & np.isin(labels, INPUT_CLASS_IDS)
        points, labels = points[valid], labels[valid]
        if not len(points):
            return False
        labels[labels == 1] = 5
        labels[labels == 6] = 4
        cells, inverse = np.unique(
            np.floor(points / self.resolution).astype(np.int64),
            axis=0, return_inverse=True)
        counts = np.stack([
            np.bincount(inverse, weights=labels == label, minlength=len(cells))
            for label in CLASS_IDS], axis=1)
        fractions = counts / counts.sum(axis=1, keepdims=True)
        self.cell_observations += len(cells)
        self.tied_observations += int(np.count_nonzero(
            (counts == counts.max(axis=1, keepdims=True)).sum(axis=1) > 1))
        tile_xy, tile_inverse = np.unique(
            cells // TILE_SIZE, axis=0, return_inverse=True)
        keys = [(key, int(x), int(y)) for x, y in tile_xy]
        needed = sum(tile_key not in self.tiles for tile_key in keys)
        if self.allocated_cells + needed * TILE_SIZE ** 2 > self.max_cells:
            raise MemoryError('Baseline exceeded the recorded max_cells limit')
        if self.sequence >= np.iinfo(np.uint32).max:
            raise OverflowError('Observation sequence exhausted')
        self.sequence += 1
        for index, tile_key in enumerate(keys):
            if tile_key not in self.tiles:
                self.tiles[tile_key] = (
                    np.zeros((TILE_SIZE ** 2, len(CLASS_IDS)), dtype=np.float32),
                    np.zeros(TILE_SIZE ** 2, dtype=np.uint32))
                self.allocated_cells += TILE_SIZE ** 2
            scores, seen = self.tiles[tile_key]
            selected = tile_inverse == index
            local = cells[selected] % TILE_SIZE
            flat = local[:, 1] * TILE_SIZE + local[:, 0]
            # Assignment is intentional: no contribution from previous clouds.
            scores[flat] = fractions[selected]
            seen[flat] = self.sequence
        return True

    def observed_mask(self, geometry):
        """Include touched-but-ambiguous cells in the future evaluation mask."""
        return observed_mask(self, geometry)


def observed_mask(grid, geometry):
    """Use cell visits, independently of either method's classification confidence."""
    mask = np.zeros(geometry.width * geometry.height, dtype=bool)
    for (key, tx, ty), (_, seen) in grid.tiles.items():
        indices = np.flatnonzero(seen)
        points = np.column_stack((
            tx * TILE_SIZE + indices % TILE_SIZE + 0.5,
            ty * TILE_SIZE + indices // TILE_SIZE + 0.5)) * grid.resolution
        points = transform_xy(points, grid.poses[key])
        target, valid = geometry.indices(points)
        mask[target[valid]] = True
    return mask.reshape(geometry.height, geometry.width)


def make_temporal_grid(config):
    """Use the production SemanticGrid implementation with the mapper's parameters."""
    grid = SemanticGrid(
        resolution=config['resolution'],
        costs=[config[name + '_cost'] for name in ('yellow', 'white', 'boardwalk')],
        hit=config['hit_log_odds'], miss=config['miss_log_odds'],
        limit=config['log_odds_limit'], threshold=config['min_evidence'],
        max_cells=config['max_cells'], max_output_cells=config['max_output_cells'],
        free_confirmations=config.get('free_confirmations', 1))
    grid.set_pose((0, 0), (0.0, 0.0, 0.0))
    return grid


class RecordedBag:
    """Read SQLite bag segments chronologically, without modifying the recording."""

    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        with (self.directory / 'metadata.yaml').open(encoding='utf-8') as stream:
            self.metadata = yaml.safe_load(stream)['rosbag2_bagfile_information']
        if self.metadata['storage_identifier'] != 'sqlite3':
            raise ValueError('This experiment expects a SQLite rosbag')
        self.files = [self.directory / name for name in self.metadata['relative_file_paths']]

    @staticmethod
    def _rows(path, topics):
        connection = sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
        try:
            definitions = {row[0]: (row[1], get_message(row[2])) for row in
                           connection.execute('SELECT id, name, type FROM topics')
                           if row[1] in topics}
            if not definitions:
                return
            placeholders = ','.join('?' for _ in definitions)
            query = ('SELECT timestamp, id, topic_id, data FROM messages WHERE topic_id IN ('
                     + placeholders + ') ORDER BY timestamp, id')
            for timestamp, message_id, topic_id, data in connection.execute(
                    query, tuple(definitions)):
                name, kind = definitions[topic_id]
                yield timestamp, message_id, name, kind, data
        finally:
            connection.close()

    def messages(self, topics):
        streams = [self._rows(path, set(topics)) for path in self.files]
        for timestamp, _, name, kind, data in heapq.merge(
                *streams, key=lambda row: (row[0], row[1])):
            yield timestamp, name, deserialize_message(data, kind)


def load_config(session):
    with (session / 'mapping_profile.yaml').open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)['semantic_mapper']
    parameters = session / 'semantic_mapper_parameters.yaml'
    if parameters.is_file():
        with parameters.open(encoding='utf-8') as stream:
            effective = yaml.safe_load(stream)
        node = next(value for name, value in effective.items()
                    if name.lstrip('/') == 'semantic_mapper')
        config.update(node['ros__parameters'])
    if config['pose_source'] != 'tf':
        raise ValueError('The unfiltered experiment currently supports pose_source=tf')
    return config


def reference_map(message, config):
    if message.header.frame_id != config['map_frame']:
        raise ValueError('Reference map frame differs from the recorded map_frame')
    geometry = Geometry(float(message.info.resolution), message.info.width, message.info.height,
                        planar_pose(message.info.origin.position, message.info.origin.orientation))
    values = np.asarray(message.data, dtype=np.int16)
    if (not np.isfinite(geometry.resolution) or geometry.resolution <= 0
            or geometry.width < 1 or geometry.height < 1
            or geometry.width * geometry.height > config['max_output_cells']
            or values.size != geometry.width * geometry.height
            or np.any(values < -1) or np.any(values > 100)):
        raise ValueError('Invalid recorded reference map')
    return geometry, values.astype(np.int8).reshape(geometry.height, geometry.width)


def write_map(stem, values, geometry, config):
    pgm = Path(str(stem) + '.pgm')
    pgm.write_bytes(map_pgm(values, 'raw'))
    Path(str(stem) + '.yaml').write_text(map_yaml(pgm.name, geometry, 'raw'), encoding='utf-8')
    # Keep cost PGM raw; use a separate RGB preview for inspecting the classes.
    palette = np.full((256, 3), 205, dtype=np.uint8)
    palette[0] = (0, 0, 255)
    palette[config['white_cost']] = (255, 255, 255)
    palette[config['yellow_cost']] = (0, 255, 255)
    palette[config['boardwalk_cost']] = (255, 0, 255)
    palette[100] = (0, 0, 0)
    indices = np.where(values >= 0, values, 255).astype(np.uint8)
    Image.fromarray(np.flipud(palette[indices])).save(str(stem) + '.png')


def file_digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def histogram(values):
    return {str(int(cost)): int(count)
            for cost, count in zip(*np.unique(values, return_counts=True))}


def reconstruct(session, output, name, temporal_filter=False, config_overrides=None):
    bag = RecordedBag(session / 'bag')
    config = load_config(session)
    recorded_config = config.copy()
    if config_overrides:
        config.update(config_overrides)
    if config['pose_source'] != 'tf':
        raise ValueError('The reconstruction supports pose_source=tf only')
    duration = bag.metadata['duration']['nanoseconds'] * 1e-9
    cache_seconds = max(3600.0, duration * 10 + 60)
    buffer = Buffer(cache_time=Duration(seconds=cache_seconds))
    statistics = Counter()
    last_tf_stamp = {}
    reference = None
    print('Pass 1: recorded transforms and final SLAM geometry...', flush=True)
    for _, topic, message in bag.messages(('/tf', '/tf_static', config['reference_map_topic'])):
        if topic == config['reference_map_topic']:
            try:
                reference = reference_map(message, config)
                statistics['reference_maps_accepted'] += 1
            except ValueError:
                statistics['reference_maps_rejected'] += 1
            continue
        for transform in message.transforms:
            key = (topic, transform.header.frame_id, transform.child_frame_id)
            stamp = stamp_ns(transform.header.stamp)
            if last_tf_stamp.get(key) == stamp:
                statistics['duplicate_tf_samples_ignored'] += 1
                continue
            last_tf_stamp[key] = stamp
            if topic == '/tf_static':
                buffer.set_transform_static(transform, 'recorded_bag')
            else:
                buffer.set_transform(transform, 'recorded_bag')
            statistics['tf_samples_loaded'] += 1
    if reference is None:
        raise ValueError('No valid SLAM map in this bag')
    geometry, laser = reference
    grid = make_temporal_grid(config) if temporal_filter else LatestMajorityGrid(config)
    last_stamp = -1
    tf_failures = []
    print('Pass 2: {}...'.format(
        'production temporal evidence filter' if temporal_filter
        else 'per-cell majority from each cloud, without temporal scores'), flush=True)
    for _, _, message in bag.messages((config['cloud_topic'],)):
        statistics['clouds_recorded'] += 1
        stamp = stamp_ns(message.header.stamp)
        if stamp <= 0 or not message.header.frame_id:
            statistics['clouds_invalid_header'] += 1
            continue
        if stamp <= last_stamp:
            statistics['clouds_nonincreasing_stamp'] += 1
            continue
        if message.width * message.height > config['max_input_points']:
            statistics['clouds_over_point_limit'] += 1
            continue
        try:
            transform = buffer.lookup_transform(
                config['map_frame'], message.header.frame_id, Time.from_msg(message.header.stamp))
            pose = planar_pose(transform.transform.translation, transform.transform.rotation)
        except (TransformException, ValueError) as error:
            statistics['clouds_without_valid_tf'] += 1
            if len(tf_failures) < 10:
                tf_failures.append({'stamp_ns': stamp, 'reason': str(error)})
            continue
        xy, labels = read_class_cloud(message)
        statistics['valid_input_points'] += len(xy)
        if grid.update(transform_xy(xy, pose), labels):
            statistics['clouds_integrated'] += 1
        else:
            statistics['clouds_without_valid_points'] += 1
        last_stamp = stamp
    if not statistics['clouds_integrated']:
        raise ValueError('No cloud could be integrated')
    _, _, semantic = grid.render(geometry)
    median_kernel = config['save_median_kernel'] if temporal_filter else 1
    if temporal_filter:
        semantic = median_saved_semantic(semantic, median_kernel)
    observed = observed_mask(grid, geometry)
    complete = combine_semantic_and_laser(semantic, laser)
    output.mkdir(parents=True, exist_ok=True)
    write_map(output / name, complete, geometry, config)
    write_map(output / (name + '_semantic'), semantic, geometry, config)
    np.savez_compressed(str(output / (name + '_data.npz')),
                        semantic=semantic, combined=complete, laser=laser, observed=observed)
    report = {
        'method': ('production_temporal_evidence' if temporal_filter
                   else 'latest_per_cloud_plurality'),
        'rule': ('Production SemanticGrid.update/render with the selected mapper parameters.'
                 if temporal_filter else
                 'Unique most-populated class per cell in the latest cloud; exact ties unknown.'),
        'label_merges': {'road': [1, 5], 'boardwalk': [4, 6]},
        'tf_policy': 'Full recorded TF history queried at each sensor timestamp.',
        'live_queue_and_wait_deadlines_simulated': False,
        'geometry_policy': 'Last valid recorded SLAM map; same sparse-grid projection as mapper.',
        'temporal_filter': temporal_filter, 'median_filter': median_kernel > 1,
        'median_kernel': median_kernel,
        'evidence_threshold': config['min_evidence'] if temporal_filter else None,
        'session': str(session), 'bag_duration_s': duration,
        'bag_sha256': {path.name: file_digest(path) for path in bag.files},
        'recorded_mapper_config': recorded_config, 'effective_mapper_config': config,
        'statistics': dict(statistics),
        'first_tf_failures': tf_failures,
        'geometry': {'width': geometry.width, 'height': geometry.height,
                     'resolution': geometry.resolution, 'origin': geometry.origin,
                     'frame': config['map_frame']},
        'cell_observations': getattr(grid, 'cell_observations', None),
        'tied_cell_observations': getattr(grid, 'tied_observations', None),
        'observed_cells': int(observed.sum()),
        'observed_but_unknown_cells': int(np.count_nonzero(observed & (semantic < 0))),
        'semantic_cost_counts': histogram(semantic), 'combined_cost_counts': histogram(complete),
    }
    (output / (name + '_report.json')).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'saved': str(output / name), 'statistics': dict(statistics),
                      'geometry': report['geometry'],
                      'semantic_cost_counts': report['semantic_cost_counts']}, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, default=DIRECTORY / 'bags/run_01')
    parser.add_argument('--output', type=Path,
                        help='Default: results/<session>/unfiltered')
    parser.add_argument('--name', default='map_unfiltered')
    args = parser.parse_args()
    if not args.name or Path(args.name).name != args.name or args.name in ('.', '..'):
        parser.error('--name must be a filename stem without a path')
    output = (args.output if args.output is not None
              else DIRECTORY / 'results' / args.session.name / 'unfiltered')
    reconstruct(args.session.resolve(), output.resolve(), args.name)


if __name__ == '__main__':
    main()
