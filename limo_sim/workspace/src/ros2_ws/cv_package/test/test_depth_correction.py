"""Exercise real/legacy ROS callbacks with calibrated synthetic depth."""

from types import SimpleNamespace
import numpy as np
import pytest

pytest.importorskip('rclpy')
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from cv_package.depth_correction import DepthCorrection
from cv_package.depth_geometry import ground_depth, pixel_rays, register_depth


@pytest.fixture
def camera():
    rclpy.init(args=['--ros-args', '-p', 'register_to_rgb:=true',
                    '-p', 'calibration_frames:=1'])
    node = DepthCorrection()
    info = CameraInfo(width=320, height=240)
    info.header.frame_id = 'depth_optical'
    info.k = [180., 0., 159.5, 0., 180., 119.5, 0., 0., 1.]
    node.camera_info_callback(info)
    rgb = CameraInfo(width=320, height=280)
    rgb.header.frame_id = 'rgb_optical'
    rgb.k = [170., 0., 165., 0., 170., 135., 0., 0., 1.]
    node.rgb_info_callback(rgb)
    rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])

    def transform(target, source, stamp):
        result = TransformStamped()
        result.transform.rotation.w = 1.
        if target == 'camera_link':
            q = result.transform.rotation
            q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
            if source == 'rgb_optical':
                result.transform.translation.y = .03
        else:
            assert target == 'rgb_optical' and source == 'depth_optical'
            result.transform.translation.x = .03
        return result

    node.tf_buffer.lookup_transform = transform
    rays = pixel_rays(info.k, 320, 240, 320, 240)
    plane = np.array([.034, .011, 1., .18])
    plane /= np.linalg.norm(plane[:3])
    depth = ground_depth(rays, rotation, np.zeros(3), plane)
    depth[170:190, 140:180] = np.nan
    msg = node.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
    msg.header.frame_id = 'depth_optical'
    msg.header.stamp.sec = 42
    published = []
    node.depth_publisher = SimpleNamespace(publish=published.append)
    node.ground_publisher = SimpleNamespace(publish=lambda msg: None)
    try:
        yield node, msg, published, plane
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_real_callback_fits_plane_registers_and_completes_every_hole(camera):
    node, msg, published, plane = camera
    node.depth_callback(msg)
    assert len(published) == 1
    result = published[0]
    assert result.header.frame_id == 'rgb_optical'
    assert result.header.stamp == msg.header.stamp
    assert (result.width, result.height) == (320, 120)
    np.testing.assert_allclose(node.ground_plane, plane, atol=1e-6)
    measured = register_depth(node.bridge.imgmsg_to_cv2(msg), node.source_rays,
                              *node.registered_transforms[0], node.rgb_info.k,
                              (320, 280), (320, 120))
    output = node.bridge.imgmsg_to_cv2(result)
    valid = np.isfinite(measured)
    np.testing.assert_array_equal(output[valid], measured[valid])
    missing = (~valid) & np.isfinite(node.registered_ground_lut)
    assert missing.sum() > 100
    np.testing.assert_array_equal(output[missing], node.registered_ground_lut[missing])


def test_calibration_change_invalidates_frozen_plane(camera):
    node, msg, _, _ = camera
    node.depth_callback(msg)
    assert node.ground_plane is not None
    node.rgb_info.k[0] += 2
    node.registered_geometry(msg)
    assert node.ground_plane is None
    assert np.isnan(node.registered_ground_lut).all()
    assert not node.ground_samples


def test_frame_or_unrectified_calibration_cannot_silently_misproject(camera):
    node, msg, published, _ = camera
    node.camera_info.header.frame_id = 'wrong_frame'
    node.depth_callback(msg)
    assert not published
    node.camera_info.header.frame_id = msg.header.frame_id
    node.rgb_info.d = [.1, 0., 0., 0., 0.]
    node.depth_callback(msg)
    assert not published


def test_default_legacy_path_preserves_simulated_geometry():
    rclpy.init(args=['--ros-args', '-p', 'plane_height_m:=-0.18'])
    node = DepthCorrection()
    try:
        assert node.register_to_rgb is False
        info = CameraInfo(width=640, height=480)
        info.header.frame_id = 'depth_optical'
        info.k = [500., 0., 319.5, 0., 500., 239.5, 0., 0., 1.]
        node.camera_info_callback(info)
        transform = TransformStamped()
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
        node.tf_buffer.lookup_transform = lambda *args: transform
        msg = node.bridge.cv2_to_imgmsg(np.ones((480,640), dtype=np.float32), '32FC1')
        msg.header.frame_id = 'depth_optical'
        msg.header.stamp.sec = 43
        published = []
        node.depth_publisher = SimpleNamespace(publish=published.append)
        node.depth_callback(msg)
        assert published[0].header == msg.header
        np.testing.assert_array_equal(node.bridge.imgmsg_to_cv2(published[0]),
                                      np.ones((120,320)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_tf_composition_roundoff_does_not_restart_calibration(camera):
    node, msg, _, _ = camera
    node.depth_callback(msg)
    plane = node.ground_plane
    original = node.tf_buffer.lookup_transform

    def noisy_transform(*args):
        transform = original(*args)
        transform.transform.translation.x += 1e-12
        return transform

    node.tf_buffer.lookup_transform = noisy_transform
    node.registered_geometry(msg)
    assert node.ground_plane is plane
    assert np.isfinite(node.registered_ground_lut).any()
