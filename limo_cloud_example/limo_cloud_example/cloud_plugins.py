"""Metric classification and reduction using this package's local algorithms."""

import numpy as np

from .processing import cloud_pipeline as legacy
from .processing.boardwalk import BoardwalkClassifier
from limo_cloud import Processor


def xy(cloud):
    return np.column_stack((cloud['x'], cloud['y']))


class Boardwalk(Processor):
    """Apply both exact nearest-neighbor passes to observed background points."""

    def configure(self, parameters):
        self.enabled = bool(parameters.get('enable_boardwalk', True))
        self.minimum = float(parameters.get('blue_radius_min_m', 0.15))
        self.maximum = float(parameters.get('blue_radius_max_m', 0.25))
        self.propagation = float(parameters.get('boardwalk_propagation_radius_m', 0.15))
        if (not np.isfinite([self.minimum, self.maximum, self.propagation]).all()
                or not 0 <= self.minimum <= self.maximum or self.propagation < 0):
            raise ValueError('Invalid boardwalk distance thresholds')
        self.classifier = BoardwalkClassifier()

    def process_cloud(self, frame):
        if self.enabled:
            frame.scratch['visual/boardwalk_stats'] = self.classifier.classify(
                xy(frame.cloud), frame.cloud['class_id'], self.minimum, self.maximum,
                self.propagation, legacy.LABEL_ROAD, legacy.LABEL_BACKGROUND,
                legacy.LABEL_BOARDWALK, legacy.LABEL_INTERIOR_BOARDWALK)

    def close(self):
        self.classifier.close()


class RoadBoardwalkOnly(Processor):
    """Apply the optional legacy output filter before metric voxelization."""

    def configure(self, parameters):
        self.enabled = bool(parameters.get('road_boardwalk_only', False))

    def process_cloud(self, frame):
        if self.enabled:
            frame.select(np.isin(frame.cloud['class_id'], (
                legacy.LABEL_ROAD, legacy.LABEL_INTERIOR_ROAD,
                legacy.LABEL_BOARDWALK, legacy.LABEL_INTERIOR_BOARDWALK)))


class MetricVoxel(Processor):
    """Use the same per-class XY centroid and representative ordering as before."""

    def configure(self, parameters):
        self.size = float(parameters.get('pointcloud_voxel_size_m', 0.02))
        if not np.isfinite(self.size) or self.size <= 0:
            raise ValueError('pointcloud_voxel_size_m must be finite and positive')

    def process_cloud(self, frame):
        if not frame.metadata['flatten_to_bev']:
            raise ValueError('The visual_ptcld compatibility pipeline requires flatten_to_bev=true')
        points, ids = legacy.voxelize_metric(xy(frame.cloud), frame.cloud['class_id'], self.size)
        xyz = np.zeros((len(points), 3), dtype=np.float32)
        xyz[:, :2] = points
        frame.set_points(xyz, ids)
