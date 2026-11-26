"""Exercise the real C++ binary over DDS using analytic ground/camera geometry."""

from array import array
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


ROOT = Path(__file__).resolve().parents[1]
ROTATION = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
PLANE = np.array([.034, .011, 1., .18])
PLANE /= np.linalg.norm(PLANE[:3])


def rays(info, width=None, height=None, crop=0.):
    width, height = width or info.width, height or info.height
    start = int(info.height * crop)
    u = (np.arange(width) + .5) * info.width / width - .5
    v = start + (np.arange(height) + .5) * (info.height - start) / height - .5
    u, v = np.meshgrid(u, v)
    return np.stack(((u - info.k[2]) / info.k[0], (v - info.k[5]) / info.k[4],
                     np.ones_like(u)), axis=-1)


def plane_depth(pixel_rays, translation=np.zeros(3), plane=PLANE):
    denominator = pixel_rays @ (ROTATION.T @ plane[:3])
    result = np.full(denominator.shape, np.nan, dtype=np.float32)
    valid = denominator < -1e-6
    result[valid] = -(plane[3] + plane[:3] @ translation) / denominator[valid]
    with np.errstate(invalid='ignore'):
        result[(result < .1) | (result > 5.)] = np.nan
    return result


def reference_registration(depth, source, target, translation, reduce):
    """Vectorized optical projection, independent of the C++ nearest-Z loop."""
    directions = rays(source)
    if reduce:
        # Solve the source-depth interval against all target frustum half-spaces.
        lower = np.full(depth.shape, .1)
        upper = np.full(depth.shape, 5.)
        constraints = [(directions[..., 2], translation[2] - .1),
                       (-directions[..., 2], 5. - translation[2])]
        for axis, lo, hi, focal, center in [
                (0, -.5, target.width - .5, target.k[0], target.k[2]),
                (1, int(target.height * .5) - .5, target.height - .5, target.k[4], target.k[5])]:
            lo, hi = (lo - center) / focal, (hi - center) / focal
            constraints += [(directions[..., axis] - lo * directions[..., 2],
                             translation[axis] - lo * translation[2]),
                            (hi * directions[..., 2] - directions[..., axis],
                             hi * translation[2] - translation[axis])]
        for coefficient, offset in constraints:
            boundary = np.zeros(depth.shape)
            np.divide(-offset, coefficient, out=boundary, where=coefficient != 0)
            np.maximum(lower, boundary, out=lower, where=coefficient > 0)
            np.minimum(upper, boundary, out=upper, where=coefficient < 0)
            upper[(coefficient == 0) & (offset < 0)] = -np.inf
        rows, cols = np.nonzero(lower <= upper)
        y0, y1 = rows.min(), rows.max() + 1
        x0, x1 = cols.min(), cols.max() + 1
        h, w = min(120, y1 - y0), min(320, x1 - x0)
        selected = np.ix_(y0 + np.arange(h) * (y1 - y0) // h,
                          x0 + np.arange(w) * (x1 - x0) // w)
        directions, depth = directions[selected], depth[selected]
    with np.errstate(invalid='ignore'):
        valid = np.isfinite(depth) & (depth >= .1) & (depth <= 5)
    points = directions[valid] * depth[valid, None] + translation
    u = (points[:, 0] / points[:, 2] * target.k[0] + target.k[2] + .5) * 320 / target.width
    v = (points[:, 1] / points[:, 2] * target.k[4] + target.k[5] - int(target.height * .5) + .5)
    v *= 120 / (target.height - int(target.height * .5))
    visible = (u >= 0) & (u < 320) & (v >= 0) & (v < 120) & (points[:, 2] >= .1) & (points[:, 2] <= 5)
    result = np.full(320 * 120, np.inf, dtype=np.float32)
    np.minimum.at(result, v[visible].astype(int) * 320 + u[visible].astype(int), points[visible, 2])
    result[~np.isfinite(result)] = np.nan
    return result.reshape(120, 320)


def decoded(message):
    dtype = '>f4' if message.is_bigendian else '<f4'
    return np.frombuffer(message.data, dtype=dtype).reshape(message.height, message.width)


class Scenario:
    def __init__(self, binary, profile, unregistered=False):
        self.node = rclpy.create_node('depth_test_' + profile + ('_native' if unregistered else ''))
        self.ns = '/' + self.node.get_name()
        self.source = CameraInfo(width=320, height=240)
        self.source.header.frame_id = self.ns[1:] + '_depth'
        self.source.k = [180., 0., 159.5, 0., 180., 119.5, 0., 0., 1.]
        self.target = CameraInfo(width=320, height=280 if profile == 'real' else 240)
        self.target.header.frame_id = self.ns[1:] + '_rgb'
        self.target.k = ([170., 0., 165., 0., 170., 135., 0., 0., 1.]
                         if profile == 'real' else list(self.source.k))
        self.rgb_translation = np.array([0., .03 if profile == 'real' else 0., 0.])
        self.unregistered = unregistered
        self.profile = profile
        self.seq = 42
        self.input = plane_depth(rays(self.source))
        self.input[170:190, 140:180] = np.nan
        self.input[135:145, 160:170] = .4  # Measured foreground must not be replaced by ground.
        self.received, self.ground, self.raw_debug, self.corrected_debug = [], [], [], []
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_pub = self.node.create_publisher(Image, self.ns + '/input', self.qos)
        self.info_pub = self.node.create_publisher(CameraInfo, self.ns + '/info', self.qos)
        self.rgb_pub = self.node.create_publisher(CameraInfo, self.ns + '/rgb_info', self.qos)
        self.subscriptions = [
            self.node.create_subscription(Image, self.ns + '/output', self.received.append, self.qos),
            self.node.create_subscription(Image, self.ns + '/ground', self.ground.append, self.qos),
            self.node.create_subscription(CompressedImage, self.ns + '/raw', self.raw_debug.append, 1),
            self.node.create_subscription(CompressedImage, self.ns + '/corrected', self.corrected_debug.append, 1),
        ]
        self.broadcaster = StaticTransformBroadcaster(self.node)
        transforms = []
        for info, translation in [(self.source, np.zeros(3)), (self.target, self.rgb_translation)]:
            transform = TransformStamped()
            transform.header.frame_id = self.ns[1:] + '_plane'
            transform.child_frame_id = info.header.frame_id
            transform.transform.rotation.x = -.5
            transform.transform.rotation.y = .5
            transform.transform.rotation.z = -.5
            transform.transform.rotation.w = .5
            transform.transform.translation.x, transform.transform.translation.y, \
                transform.transform.translation.z = translation.tolist()
            transforms.append(transform)
        self.broadcaster.sendTransform(transforms)
        args = [binary, '--ros-args', '--params-file', str(ROOT / 'config/depth.yaml'),
                '--params-file', str(ROOT / ('config/' + profile + '.yaml'))]
        overrides = dict(use_sim_time='false', calibration_frames='2',
                         plane_frame=self.ns[1:] + '_plane',
                         input_topic=self.ns + '/input', camera_info_topic=self.ns + '/info',
                         rgb_camera_info_topic=self.ns + '/rgb_info', depth_output_topic=self.ns + '/output',
                         ground_depth_topic=self.ns + '/ground', output_topic=self.ns + '/raw',
                         corrected_output_topic=self.ns + '/corrected')
        if unregistered:
            overrides.update(register_to_rgb='false', plane_height_m='-0.18')
        for key, value in overrides.items():
            args += ['-p', 'limo_depth:{}:={}'.format(key, value)]
        self.output = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen(args, stdout=self.output, stderr=subprocess.STDOUT)
        self.parameters = self.node.create_client(SetParametersAtomically, '/limo_depth/set_parameters_atomically')
        self.reset = self.node.create_client(Trigger, '/limo_depth/reset_calibration')

    def logs(self):
        self.output.seek(0)
        return self.output.read()

    def spin(self, duration=.1):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(.02, max(0, deadline - time.monotonic())))
            assert self.process.poll() is None, self.logs()

    def wait(self, predicate, action=None, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if action:
                action()
            self.spin(.08)
        assert predicate(), self.logs()

    def send(self, invalid=False, big_endian_mm=False, calibration=True):
        self.seq += 1
        message = Image(width=320, height=240, encoding='32FC1', step=320 * 4 + 8)
        message.header.frame_id = self.source.header.frame_id
        message.header.stamp.sec = self.seq
        if big_endian_mm:
            message.encoding, message.is_bigendian, message.step = '16UC1', 1, 320 * 2 + 8
            depth = np.rint(np.nan_to_num(self.input) * 1000).astype('>u2')
        else:
            depth = self.input.astype('<f4')
        padded = np.zeros((240, message.step), dtype=np.uint8)
        padded[:, :depth.shape[1] * depth.dtype.itemsize] = depth.view(np.uint8).reshape(240, -1)
        message.data = array('B', padded.tobytes())
        if invalid:
            message.step = 1
        if calibration:
            self.info_pub.publish(self.source)
            self.rgb_pub.publish(self.target)
        self.depth_pub.publish(message)
        return message

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=5), self.logs()
        future = client.call_async(request)
        self.wait(future.done)
        return future.result()

    def set(self, values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(key, value=value).to_parameter_msg() for key, value in values.items()]
        return self.call(self.parameters, request).result

    def check_result(self, message, depth=None):
        depth = self.input if depth is None else depth
        assert (message.width, message.height, message.encoding) == (320, 120, '32FC1')
        target = self.source if self.unregistered else self.target
        assert message.header.frame_id == target.header.frame_id
        assert 42 < message.header.stamp.sec <= self.seq
        if self.unregistered:
            expected = depth[120:, :].copy()
            with np.errstate(invalid='ignore'):
                expected[(expected < .1) | (expected > 5.)] = np.nan
            ground = plane_depth(rays(self.source, 320, 120, .5), plane=np.array([0, 0, 1, .18]))
        else:
            translation = ROTATION.T @ -self.rgb_translation
            expected = reference_registration(depth, self.source, self.target, translation, self.profile == 'real')
            ground = plane_depth(rays(self.target, 320, 120, .5), self.rgb_translation)
        valid = np.isfinite(expected)
        actual = decoded(message)
        np.testing.assert_allclose(actual[valid], expected[valid], atol=1e-6)
        missing = ~valid & np.isfinite(ground)
        assert missing.sum() > 100
        np.testing.assert_allclose(actual[missing], ground[missing], atol=2e-6)
        expected[missing] = ground[missing]
        np.testing.assert_allclose(actual, expected, atol=2e-6, equal_nan=True)

    def run(self):
        self.wait(lambda: self.depth_pub.get_subscription_count() > 0)
        # Without CameraInfo, malformed data and missing geometry cannot publish.
        for _ in range(3):
            self.send(calibration=False)
            self.spin(.08)
        assert not self.received
        self.wait(lambda: self.info_pub.get_subscription_count() > 0)
        for _ in range(3):
            self.send(invalid=True)
            self.spin(.08)
        assert not self.received
        self.wait(lambda: bool(self.ground and np.isfinite(decoded(self.ground[-1])).any()), self.send)
        self.wait(lambda: bool(self.received and self.raw_debug and self.corrected_debug), self.send)
        self.check_result(self.received[-1])
        assert bytes(self.raw_debug[-1].data)[:2] == b'\xff\xd8'
        assert bytes(self.corrected_debug[-1].data)[:2] == b'\xff\xd8'

        # Frozen ground must coexist with padded, big-endian millimetre ingress.
        self.spin(.15)
        previous = self.received[-1].header.stamp.sec
        self.wait(lambda: self.received[-1].header.stamp.sec > previous,
                  lambda: self.send(big_endian_mm=True))
        quantized = np.rint(np.nan_to_num(self.input) * 1000).astype(np.uint16).astype(np.float32) * np.float32(.001)
        self.check_result(self.received[-1], quantized)

        # Reject an atomic invalid update without applying the telemetry change.
        result = self.set(dict(max_processing_rate_hz=-1., enable_telemetry=True))
        assert not result.successful
        result = self.set(dict(output_width=10))
        assert not result.successful, 'Startup-only geometry parameter accepted'
        assert self.set(dict(max_processing_rate_hz=10., enable_telemetry=True)).successful
        self.spin(.2)
        count = len(self.received)
        started = time.monotonic()
        while time.monotonic() - started < .65:
            self.send()
            self.spin(.01)
        self.spin(.2)  # Drain one final pending image.
        assert 4 <= len(self.received) - count <= 9, len(self.received) - count
        count = len(self.received)
        self.spin(.22)
        assert len(self.received) == count, 'Idle timer republished a consumed frame'
        assert self.set(dict(max_processing_rate_hz=0.)).successful

        # Reset erases the fitted LUT; one good frame is insufficient for a 2-frame fit.
        assert self.call(self.reset, Trigger.Request()).success
        self.spin(.1)
        self.ground.clear()
        stamp = self.send().header.stamp.sec
        self.wait(lambda: bool(self.ground and self.ground[-1].header.stamp.sec == stamp))
        if not self.unregistered:
            assert np.isnan(decoded(self.ground[-1])).all()
        self.wait(lambda: np.isfinite(decoded(self.ground[-1])).any(), self.send)

        # An intrinsic update also invalidates the frozen ground fit.
        if not self.unregistered:
            self.target.k[0] += 2.
            self.rgb_pub.publish(self.target)
            self.spin(.1)
            stamp = self.send().header.stamp.sec
            self.wait(lambda: self.ground[-1].header.stamp.sec == stamp)
            assert np.isnan(decoded(self.ground[-1])).all()
            self.wait(lambda: np.isfinite(decoded(self.ground[-1])).any(), self.send)
            self.check_result(self.received[-1])
            # A changed source frame with no matching image frame must be rejected.
            original_frame = self.source.header.frame_id
            self.source.header.frame_id = 'incorrect_source_frame'
            self.info_pub.publish(self.source)
            self.spin(.1)
            count = len(self.received)
            message = self.send(calibration=False)
            message.header.frame_id = original_frame
            self.depth_pub.publish(message)
            self.spin(.12)
            assert len(self.received) == count
            self.source.header.frame_id = original_frame
            self.info_pub.publish(self.source)
            # Distorted or mismatched calibration must not silently project.
            self.target.d = [.1, 0., 0., 0., 0.]
            self.rgb_pub.publish(self.target)
            self.spin(.15)
            count = len(self.received)
            for _ in range(3):
                self.send()
                self.spin(.08)
            assert len(self.received) == count
        print('DDS {} {}: geometry, completion, endian/padding, reset, parameters, rate and JPEG OK'.format(
            self.profile, 'native' if self.unregistered else 'registered'))

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        logs = self.logs()
        self.node.destroy_node()
        self.output.close()
        assert self.process.returncode == 0, logs


if __name__ == '__main__':
    rclpy.init()
    try:
        for profile, native in [('sim', False), ('real', False), ('sim', True)]:
            scenario = Scenario(sys.argv[1], profile, native)
            try:
                scenario.run()
            finally:
                scenario.close()
    finally:
        rclpy.shutdown()
