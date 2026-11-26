"""Plugins use process_images(frame) and/or process_cloud(frame), once per frame.

Inputs are read-only NumPy views over owned ROS buffers. Assign a new array or
call edit_labels()/edit_depth() to modify an image. The outgoing cloud and named
intermediates are writable. Replacing/selecting a cloud allocates a new buffer;
retained views continue to own the old data. Prefer namespaced intermediate keys.
"""

import numpy as np


POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'class_id'],
    'formats': ['=f4', '=f4', '=f4', 'u1'],
    'offsets': [0, 4, 8, 12],
    'itemsize': 16,
})

SAMPLE_DTYPE = np.dtype({
    'names': ['row', 'col', 'class_id'],
    'formats': ['=u4', '=u4', 'u1'],
    'offsets': [0, 4, 8],
    'itemsize': 12,
})


class Processor:
    """Subclass in any importable module, then configure python:module:Class."""

    def configure(self, parameters):
        self.parameters = parameters

    def process_images(self, frame):
        pass

    def process_cloud(self, frame):
        pass

    def close(self):
        """Release optional resources when the host destroys this plugin."""
        pass


class Frame:
    """Shared stage context; images/clouds carry cross-language intermediates.

    scratch accepts arbitrary Python values for the duration of one frame.
    metadata contains camera/TF/crop information. Changing the original camera
    geometry through a plugin is intentionally unsupported. set_samples() requests
    a sparse ordered projection with explicit IDs; None retains the dense path.
    close() is called on plugin destruction, after processing has stopped.
    """

    def __init__(self, depth, labels, cloud, images, clouds, metadata, scratch, samples=None):
        self.depth = depth
        self.labels = labels
        self.cloud = cloud
        # Plugins that establish finiteness can explicitly declare a dense cloud.
        self.cloud_is_dense = False
        self.images = images
        self.clouds = clouds
        self.metadata = metadata
        self.scratch = scratch
        # None projects every valid pixel; an empty array projects no points.
        # Ordered explicit IDs allow several classes to sample the same pixel.
        self.samples = samples

    def set_samples(self, pixels, class_ids):
        """Request ordered (row, column) pixels with explicit uint8 class IDs."""
        pixels = np.asarray(pixels)
        class_ids = np.asarray(class_ids)
        if (pixels.ndim != 2 or pixels.shape[1] != 2
                or class_ids.shape != (len(pixels),)):
            raise ValueError('Expected pixels shape (N, 2) and class_ids shape (N,)')
        if (not np.issubdtype(pixels.dtype, np.integer)
                or np.any(pixels < 0)
                or np.any(pixels[:, 0] >= self.labels.shape[0])
                or np.any(pixels[:, 1] >= self.labels.shape[1])):
            raise ValueError('Sample pixels must be integer coordinates within the image')
        if (not np.issubdtype(class_ids.dtype, np.integer)
                or np.any(class_ids < 0) or np.any(class_ids > 255)):
            raise ValueError('Sample class IDs must be integers in [0, 255]')
        samples = np.zeros(len(pixels), dtype=SAMPLE_DTYPE)
        samples['row'], samples['col'] = pixels.T
        samples['class_id'] = class_ids
        self.samples = samples

    def edit_labels(self):
        if not self.labels.flags.writeable:
            self.labels = self.labels.copy()
        return self.labels

    def edit_depth(self):
        if not self.depth.flags.writeable:
            self.depth = self.depth.copy()
        return self.depth

    def select(self, selection):
        """Keep selected points, preserving coordinates and all class IDs."""
        if self.cloud is None:
            raise ValueError('Point selection requires the cloud stage')
        self.cloud = self.cloud[selection].copy()

    def set_points(self, xyz, class_ids):
        """Replace the cloud with Nx3 coordinates and N uint8 class IDs."""
        xyz = np.asarray(xyz)
        class_ids = np.asarray(class_ids)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or class_ids.shape != (len(xyz),):
            raise ValueError('Expected XYZ shape (N, 3) and class_ids shape (N,)')
        if (not np.issubdtype(class_ids.dtype, np.integer)
                or np.any(class_ids < 0) or np.any(class_ids > 255)):
            raise ValueError('class_ids must contain integers in [0, 255]')
        cloud = np.zeros(len(xyz), dtype=POINT_DTYPE)
        for axis, name in enumerate(('x', 'y', 'z')):
            cloud[name] = xyz[:, axis]
        cloud['class_id'] = class_ids
        self.cloud = cloud
        self.cloud_is_dense = bool(np.isfinite(xyz).all())
