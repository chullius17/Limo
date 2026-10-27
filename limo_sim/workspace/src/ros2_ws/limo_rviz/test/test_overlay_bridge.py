# Copyright 2026 Giulio Cataldo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify simulation and real RViz profiles decode JPEG lane overlays."""

from pathlib import Path
import signal
import subprocess
import time

import pytest
import yaml

from limo_rviz.overlay import LANE_OVERLAY, WATERFALL_OVERLAY, lane_overlay_bridge


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('profile,viewer,topic', [
    ('online_map_real.rviz', 'online_rviz', WATERFALL_OVERLAY),
    ('offline_map_real.rviz', 'offline_rviz', WATERFALL_OVERLAY),
    ('cv_visual_real.rviz', 'cv_rviz', WATERFALL_OVERLAY),
    ('online_map.rviz', 'online_rviz', LANE_OVERLAY),
    ('offline_map.rviz', 'offline_rviz', LANE_OVERLAY),
    ('cv_visual.rviz', 'cv_rviz', LANE_OVERLAY),
])
def test_viewers_wire_compressed_overlay_to_raw_display(profile, viewer, topic):
    decoder, remappings = lane_overlay_bridge(PACKAGE / 'config' / profile, viewer)
    assert decoder['arguments'] == ['compressed', 'raw']
    assert dict(decoder['remappings'])['in/compressed'] == topic + '/compressed'
    assert dict(remappings)[topic] == dict(decoder['remappings'])['out']


def test_disabled_overlay_does_not_start_decoder(tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/online_map_real.rviz').read_text())
    for display in config['Visualization Manager']['Displays']:
        if display['Class'] == 'rviz_default_plugins/Image':
            display['Enabled'] = False
    path = tmp_path / 'disabled.rviz'
    path.write_text(yaml.safe_dump(config))
    assert lane_overlay_bridge(path, 'test') == (None, [])


@pytest.mark.parametrize('profile,topic', [
    ('online_map.rviz', LANE_OVERLAY),
    ('online_map_real.rviz', WATERFALL_OVERLAY),
])
def test_republisher_decodes_jpeg_and_preserves_header(profile, topic):
    rclpy = pytest.importorskip('rclpy')
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    from ament_index_python.packages import get_package_prefix
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CompressedImage, Image

    decoder, remappings = lane_overlay_bridge(
        PACKAGE / 'config' / profile, 'overlay_test')
    command = [get_package_prefix('image_transport') + '/lib/image_transport/republish',
               *decoder['arguments'], '--ros-args', '-r', '__node:=' + decoder['name']]
    for source, destination in decoder['remappings']:
        command.extend(['-r', source + ':=' + destination])
    rclpy.init(args=[])
    node = rclpy.create_node('overlay_bridge_test')
    received = []
    node.create_subscription(Image, dict(remappings)[topic], received.append, 10)
    publisher = node.create_publisher(CompressedImage, topic + '/compressed', 10)
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        pixels = np.full((16, 32, 3), (200, 20, 10), dtype=np.uint8)
        success, encoded = cv2.imencode('.jpg', pixels)
        assert success
        message = CompressedImage(format='jpeg', data=encoded.tobytes())
        message.header.frame_id = 'camera_optical_frame'
        message.header.stamp.sec = 123
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            assert process.poll() is None
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received, 'No raw image received from the compressed overlay decoder'
        image = received[-1]
        assert image.header == message.header
        assert (image.width, image.height) == (32, 16)
        decoded = CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8')
        assert np.max(np.abs(decoded.astype(int) - pixels.astype(int))) <= 3
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()
