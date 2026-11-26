"""Run the actual legacy node methods with deterministic sensors and publishers."""

from pathlib import Path
import math
import threading
from types import SimpleNamespace

import numpy as np
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

from cv_package.visual_ptcld import cloud_pipeline as legacy
from cv_package.visual_ptcld.boardwalk import BoardwalkClassifier
from cv_package.visual_ptcld.cloud_cpu import RayCache, transform_xy
from cv_package.visual_ptcld.visual_ptcld import VisualPtcld
from limo_cloud.api import Frame, POINT_DTYPE
from limo_cloud._runtime import has_hook, load
from limo_cloud_example.configuration import load_parameters
from limo_cloud_example.cv_configuration import cloud_parameters

PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent.parent

DEFAULT_TYPES = {
    'masks': 'python:limo_cloud_example.image_plugins:BoundaryMasks',
    'sampling': 'python:limo_cloud_example.image_plugins:PixelSampling',
    'debug': 'python:limo_cloud_example.debug:DebugOutputs',
    'boardwalk': 'python:limo_cloud_example.cloud_plugins:Boardwalk',
    'class_filter': 'python:limo_cloud_example.cloud_plugins:RoadBoardwalkOnly',
    'voxel': 'python:limo_cloud_example.cloud_plugins:MetricVoxel',
}


def default_parameters():
    parameters = {'plugins': list(DEFAULT_TYPES)}
    parameters.update({name + '.type': value for name, value in DEFAULT_TYPES.items()})
    parameters.update(load_parameters(PACKAGE / 'config/cloud.yaml'))
    return parameters


def settings(profile='sim', **overrides):
    original = yaml.safe_load((PACKAGES / 'cv_package/config' / ('cv_' + profile + '.yaml')).read_text())
    original['visual_ptcld'].update(overrides)
    parameters = default_parameters()
    parameters.update(load_parameters(PACKAGE / 'config' / (profile + '.yaml')))
    parameters.update(cloud_parameters(original['visual_ptcld']))
    # Rendering is tested separately; no ROS context is needed by numerical tests.
    parameters['debug.enable_debug_publications'] = False
    return original['visual_ptcld'], parameters


def scene(seed=9, shape=(24, 64)):
    rng = np.random.RandomState(seed)
    height, width = shape
    labels = rng.choice([0, 1, 2, 3, 255], shape, p=[.05, .35, .10, .48, .02]).astype(np.uint8)
    labels[height//4:3*height//4, width//4:3*width//4] = 1
    labels[height//3:2*height//3, width//2] = 2
    depth = rng.uniform(.08, 2.6, shape).astype(np.float32)
    depth.flat[:6] = [0, np.nan, np.inf, -.2, .1, 2.5]
    return labels, depth


def transform():
    result = TransformStamped()
    result.header.frame_id = 'base_link'
    result.child_frame_id = 'test_rgb_optical'
    result.transform.rotation.x = .1
    result.transform.rotation.y = -.17
    result.transform.rotation.z = .29
    result.transform.rotation.w = .9
    q = result.transform.rotation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    result.transform.translation.x = .24
    result.transform.translation.y = -.13
    result.transform.translation.z = .12
    return result


INTRINSICS = (513.3, 514.2, 319.4, 241.2, 640, 480)


def legacy_output(labels, depth, parameters, debug_outputs=None):
    bridge = CvBridge()
    header = Header(frame_id='test_rgb_optical')
    header.stamp.sec = 42
    published = []
    logger = SimpleNamespace(**{name: lambda *args, **kwargs: None
                                for name in ('warn', 'warning', 'error')})
    detector = SimpleNamespace(
        bridge=bridge, sensor_lock=threading.Lock(),
        depth_image=depth.astype(np.float32) * np.float32(.001)
        if depth.dtype.kind == 'u' else depth,
        depth_frame=header.frame_id, camera_info_frame=header.frame_id,
        camera_intrinsics=INTRINSICS, require_rgb_depth=True,
        cloud_min_depth=parameters.get('pointcloud_min_depth_m', .1),
        cloud_max_depth=parameters.get('pointcloud_max_depth_m', 2.5),
        input_crop_y_min=parameters.get('input_crop_y_min', .5),
        roi_y_min=parameters.get('roi_y_min', 0.), roi_y_max=parameters.get('roi_y_max', 1.),
        bev_frame='base_link', enable_boardwalk=parameters.get('enable_boardwalk', True),
        road_boardwalk_only=parameters.get('road_boardwalk_only', False),
        enable_debug_publications=False,
        blue_radius_min=parameters.get('blue_radius_min_m', .15),
        blue_radius_max=parameters.get('blue_radius_max_m', .25),
        boardwalk_propagation_radius=parameters.get('boardwalk_propagation_radius_m', .15),
        pointcloud_voxel_size=parameters.get('pointcloud_voxel_size_m', .02),
        blue_boundary_kernel=np.ones((parameters.get('blue_boundary_kernel_size', 7),) * 2, np.uint8),
        point_voxel_size=parameters.get('point_voxel_size', 3),
        ray_cache=RayCache(), boardwalk_classifier=BoardwalkClassifier(),
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args: transform()),
        quaternion_to_rotation=VisualPtcld.quaternion_to_rotation,
        pointcloud_pub=SimpleNamespace(publish=published.append),
        get_logger=lambda: logger,
        log_diagnostics=lambda *args: None,
    )
    for name in dir(VisualPtcld):
        if name.startswith('LABEL_'):
            setattr(detector, name, getattr(VisualPtcld, name))
    detector.voxelize_points = lambda pixels, width: VisualPtcld.voxelize_points(detector, pixels, width)
    detector.publish_pointcloud = lambda *args, **kwargs: VisualPtcld.publish_pointcloud(detector, *args, **kwargs)
    if debug_outputs is not None:
        from turbojpeg import TurboJPEG
        detector.enable_debug_publications = True
        detector.jpeg = TurboJPEG()
        detector.debug_jpeg_quality = parameters.get('debug_jpeg_quality', 85)
        for attribute, key in (('before_kdtree_pub', 'before'), ('debug_pub', 'debug'),
                               ('lines_pub', 'lines'), ('blue_filter_debug_pub', 'filter')):
            captured = debug_outputs.setdefault(key, [])
            setattr(detector, attribute, SimpleNamespace(
                get_subscription_count=lambda: 1, publish=captured.append))
        detector.encode_debug_image = lambda image, header: VisualPtcld.encode_debug_image(detector, image, header)
    message = bridge.cv2_to_imgmsg(labels, 'mono8')
    message.header = header
    VisualPtcld.process_image(detector, message)
    assert len(published) == 1
    return published[0]


def frame_for(labels, depth, parameters):
    fx, fy, cx, cy, width, height = INTRINSICS
    t = transform().transform
    metadata = {
        'roi_y_min': parameters.get('roi_y_min', 0.),
        'roi_y_max': parameters.get('roi_y_max', 1.),
        'input_crop_y_min': parameters.get('input_crop_y_min', .5),
        'flatten_to_bev': True,
        'k': [fx, 0., cx, 0., fy, cy, 0., 0., 1.],
        'camera_width': width, 'camera_height': height,
        'translation': [t.translation.x, t.translation.y, t.translation.z],
        'rotation_xyzw': [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
        'stamp_ns': 42 * 10**9, 'frame_id': 'test_rgb_optical',
        'target_frame': 'base_link', 'node_namespace': '/',
    }
    return Frame(depth, labels, None, {}, {}, metadata, {})


def plugins(parameters):
    result = []
    for name in parameters['plugins']:
        prefix = name + '.'
        config = {key[len(prefix):]: value for key, value in parameters.items()
                  if key.startswith(prefix) and key != prefix + 'type'}
        result.append(load(parameters[prefix + 'type'][7:], config))
    return result


def project_reference(frame, parameters):
    samples = frame.samples
    rows, cols = samples['row'], samples['col']
    depth = frame.depth
    z = depth[rows, cols].astype(np.float32)
    if depth.dtype.kind == 'u':
        z *= np.float32(.001)
    valid = (np.isfinite(z) & (z >= parameters.get('pointcloud_min_depth_m', .1))
             & (z <= parameters.get('pointcloud_max_depth_m', 2.5)))
    rows, cols, z = rows[valid], cols[valid], z[valid]
    height, width = depth.shape
    ray_x, ray_y = RayCache().get(INTRINSICS, width, height, frame.metadata['input_crop_y_min'])
    tf = transform().transform
    rotation = legacy.quaternion_to_rotation(tf.rotation)
    xy = transform_xy(ray_x[cols] * z, ray_y[rows] * z, z, rotation,
                      (tf.translation.x, tf.translation.y))
    cloud = np.zeros(len(z), dtype=POINT_DTYPE)
    cloud['x'], cloud['y'] = xy.T
    cloud['class_id'] = samples['class_id'][valid]
    frame.cloud = cloud


def run_plugins(labels, depth, parameters):
    chain = plugins(parameters)
    frame = frame_for(labels, depth, parameters)
    try:
        for plugin in chain:
            if has_hook(plugin, 'process_images'):
                plugin.process_images(frame)
        project_reference(frame, parameters)
        for plugin in chain:
            if has_hook(plugin, 'process_cloud'):
                plugin.process_cloud(frame)
        return frame
    finally:
        for plugin in reversed(chain):
            plugin.close()
