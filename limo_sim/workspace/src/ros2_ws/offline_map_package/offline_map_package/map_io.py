"""Nav2 map serialization shared by the ROS mapper and offline experiments."""

import numpy as np


def map_pgm(values, mode):
    """Encode costs as a Nav2 raw-, scale- or trinary-mode binary PGM."""
    if values.ndim != 2:
        raise ValueError('Saved map must be a two-dimensional array')
    known = values >= 0
    if np.any(values < -1) or np.any(values[known] > 100):
        raise ValueError('Map costs must be between 0 and 100')
    if mode == 'raw':
        # Nav2 raw mode reads 0..100 directly; 255 represents unknown.
        pixels = np.where(known, values, 255).astype(np.uint8)
    elif mode == 'scale':
        # Scale grayscale PGM cannot distinguish unknown from a cost.
        if not np.all(known):
            raise ValueError('Scale PGM cannot encode unknown cells')
        pixels = np.rint(
            (100.0 - values.astype(np.float64)) * (255.0 / 100.0)
        ).astype(np.uint8)
    elif mode == 'trinary':
        pixels = np.full(values.shape, 205, dtype=np.uint8)
        pixels[known & (values <= 25)] = 254
        pixels[known & (values >= 65)] = 0
    else:
        raise ValueError('Map mode must be raw, scale or trinary')
    # OccupancyGrid starts at the lower-left; image rows start at the upper-left.
    pixels = np.flipud(pixels)
    header = 'P5\n# CREATOR: offline_map_package semantic_mapper\n{} {}\n255\n'.format(
        values.shape[1], values.shape[0])
    return header.encode('ascii') + pixels.tobytes()


def map_yaml(image_name, geometry, mode):
    """Create metadata accepted by the Foxy/Humble Nav2 map server."""
    if mode in ('raw', 'scale'):
        # Raw mode ignores thresholds. Scale mode uses the full [0, 1]
        # interval to reconstruct costs without the default threshold stretch.
        occupied_thresh, free_thresh = '1.0', '0.0'
    elif mode == 'trinary':
        # These thresholds separate the conventional 0/205/254 PGM pixels.
        occupied_thresh, free_thresh = '0.65', '0.196'
    else:
        raise ValueError('Map mode must be raw, scale or trinary')
    return (
        'image: {}\n'
        'mode: {}\n'
        'resolution: {:.17g}\n'
        'origin: [{:.17g}, {:.17g}, {:.17g}]\n'
        'negate: 0\n'
        'occupied_thresh: {}\n'
        'free_thresh: {}\n'
    ).format(image_name, mode, geometry.resolution, *geometry.origin,
             occupied_thresh, free_thresh)
