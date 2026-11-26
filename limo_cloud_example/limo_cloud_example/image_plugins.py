"""Image masks and ordered pixel sampling, matching visual_ptcld exactly."""

import cv2
import numpy as np

from .processing import cloud_pipeline as legacy
from .processing.boardwalk import BoardwalkClassifier
from limo_cloud import Processor


class BoundaryMasks(Processor):
    """Split the road at the soft-obstacle boundary, then apply the output ROI."""

    def configure(self, parameters):
        threads = int(parameters.get('opencv_num_threads', 1))
        size = int(parameters.get('blue_boundary_kernel_size', 7))
        if threads < 1 or size < 1 or size % 2 != 1:
            raise ValueError('OpenCV threads must be positive; boundary kernel must be positive and odd')
        cv2.setNumThreads(threads)
        self.kernel = np.ones((size, size), dtype=np.uint8)

    def process_images(self, frame):
        labels = frame.labels
        # Adjacency is computed before ROI cropping, as in the original node.
        boundary = BoardwalkClassifier.filter_blue(
            labels, self.kernel, legacy.LABEL_ROAD, legacy.LABEL_BACKGROUND)
        interior = (labels == legacy.LABEL_ROAD) & ~boundary
        roi = np.zeros(labels.shape, dtype=bool)
        height = labels.shape[0]
        roi[int(height * frame.metadata['roi_y_min']):
            int(height * frame.metadata['roi_y_max']), :] = True
        masks = {
            'road': boundary,
            'yellow': labels == legacy.LABEL_YELLOW,
            'background': labels == legacy.LABEL_BACKGROUND,
            'interior_road': interior,
        }
        for name, mask in masks.items():
            frame.images['visual/' + name] = (mask & roi).astype(np.uint8)


class PixelSampling(Processor):
    """Sample each legacy mask without deduplicating pixels or changing order."""

    def configure(self, parameters):
        self.size = int(parameters.get('point_voxel_size', 3))
        self.passthrough_extra_classes = bool(parameters.get('passthrough_extra_classes', True))
        if self.size <= 0:
            raise ValueError('point_voxel_size must be positive')

    def process_images(self, frame):
        width = frame.labels.shape[1]
        groups = []
        for name, label in (
                ('road', legacy.LABEL_ROAD), ('yellow', legacy.LABEL_YELLOW),
                ('background', legacy.LABEL_BACKGROUND),
                ('interior_road', legacy.LABEL_INTERIOR_ROAD)):
            mask = frame.images['visual/' + name]
            pixels = np.argwhere(mask)
            if label != legacy.LABEL_YELLOW:
                pixels = legacy.voxelize_pixels(pixels, width, self.size)
            # Only interior-road representatives are checked against their source
            # mask. Boundary/background centroids may fall outside nonconvex masks.
            if label == legacy.LABEL_INTERIOR_ROAD:
                pixels = pixels[mask[tuple(pixels.T)] != 0]
            groups.append((pixels, label))
        frame.scratch['visual/point_groups'] = groups
        if self.passthrough_extra_classes:
            # Additional input IDs bypass the legacy 1/2/3 mask-specific sampling.
            # They still use the same ROI, C++ depth validation and projection.
            extra = frame.labels > legacy.LABEL_BACKGROUND
            height = extra.shape[0]
            extra[:int(height * frame.metadata['roi_y_min'])] = False
            extra[int(height * frame.metadata['roi_y_max']):] = False
            pixels = np.argwhere(extra)
            ids = frame.labels[tuple(pixels.T)]
        else:
            pixels = np.empty((0, 2), dtype=np.int32)
            ids = np.empty(0, dtype=np.uint8)
        pixels = np.concatenate([p for p, _ in groups] + [pixels], axis=0)
        ids = np.concatenate([
            np.full(len(p), label, dtype=np.uint8) for p, label in groups] + [ids])
        frame.set_samples(pixels, ids)
