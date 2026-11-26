"""Internal bridge; the C++ exporter owns each buffer exposed to NumPy."""

import importlib
import sys

import numpy as np

from .api import Frame, POINT_DTYPE, SAMPLE_DTYPE, Processor


def load(specification, parameters):
    module, name = specification.split(':', 1)
    plugin = getattr(importlib.import_module(module), name)()
    configure = getattr(plugin, 'configure', None)
    if configure is not None:
        configure(parameters)
    return plugin


def has_hook(plugin, name):
    hook = getattr(plugin, name, None)
    return callable(hook) and getattr(type(plugin), name, None) is not getattr(Processor, name)


def image_array(buffer, height, width, step, encoding, bigendian):
    formats = {'mono8': 'u1', '8UC1': 'u1', 'mono16': 'u2',
               '16UC1': 'u2', '32FC1': 'f4'}
    dtype = np.dtype(('>' if bigendian else '<') + formats[encoding])
    return np.ndarray((height, width), dtype=dtype, buffer=buffer,
                      strides=(step, dtype.itemsize))


def cloud_array(buffer, width):
    return np.ndarray((width,), dtype=POINT_DTYPE, buffer=buffer)


def sample_array(buffer, count):
    return np.ndarray((count,), dtype=SAMPLE_DTYPE, buffer=buffer)


def normalise_samples(array):
    array = np.asarray(array)
    if array.ndim != 1 or array.dtype != SAMPLE_DTYPE:
        raise ValueError('Samples must be 1D with limo_cloud.SAMPLE_DTYPE')
    return np.ascontiguousarray(array)


def normalise_image(array):
    array = np.asarray(array)
    encoding = {('u', 1): 'mono8', ('u', 2): '16UC1', ('f', 4): '32FC1'}.get(
        (array.dtype.kind, array.dtype.itemsize))
    if array.ndim != 2 or not all(array.shape) or encoding is None:
        raise ValueError('Intermediate images must be nonempty 2D uint8/uint16/float32 arrays')
    return np.ascontiguousarray(array), encoding, (
        array.dtype.byteorder == '>' or
        (array.dtype.byteorder == '=' and sys.byteorder == 'big'))


def normalise_cloud(array):
    array = np.asarray(array)
    if array.ndim != 1 or array.dtype != POINT_DTYPE:
        raise ValueError('Cloud arrays must be 1D with limo_cloud.POINT_DTYPE')
    return np.ascontiguousarray(array)
