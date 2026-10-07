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

"""Verify real RViz profiles decode JPEG overlays into raw Image messages."""

from pathlib import Path
import signal
import subprocess
import time

import pytest
import yaml

from limo_rviz.overlay import WATERFALL_OVERLAY, waterfall_overlay_bridge


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('profile,viewer', [
    ('online_map_real.rviz', 'online_rviz'),
    ('offline_map_real.rviz', 'offline_rviz'),
    ('cv_visual_real.rviz', 'cv_rviz'),
])
def test_real_viewers_wire_compressed_overlay_to_raw_display(profile, viewer):
    decoder, remappings = waterfall_overlay_bridge(PACKAGE / 'config' / profile, viewer)
    assert decoder['arguments'] == ['compressed', 'raw']
    assert dict(decoder['remappings'])['in/compressed'] == WATERFALL_OVERLAY + '/compressed'
    assert dict(remappings)[WATERFALL_OVERLAY] == dict(decoder['remappings'])['out']


def test_disabled_overlay_does_not_start_decoder(tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/online_map_real.rviz').read_text())
    for display in config['Visualization Manager']['Displays']:
        if display['Class'] == 'rviz_default_plugins/Image':
            display['Enabled'] = False
    path = tmp_path / 'disabled.rviz'
    path.write_text(yaml.safe_dump(config))
    assert waterfall_overlay_bridge(path, 'test') == (None, [])


def test_republisher_decodes_jpeg_and_preserves_header():
    rclpy = pytest.importorskip('rclpy')
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    from ament_index_python.packages import get_package_prefix
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CompressedImage, Image

    decoder, remappings = waterfall_overlay_bridge(
        PACKAGE / 'config/online_map_real.rviz', 'overlay_test')
    command = [get_package_prefix('image_transport') + '/lib/image_transport/republish',
               *decoder['arguments'], '--ros-args', '-r', '__node:=' + decoder['name']]
    for source, destination in decoder['remappings']:
        command.extend(['-r', source + ':=' + destination])
    rclpy.init(args=[])
    node = rclpy.create_node('overlay_bridge_test')
    received = []
    node.create_subscription(Image, dict(remappings)[WATERFALL_OVERLAY], received.append, 10)
    publisher = node.create_publisher(CompressedImage, WATERFALL_OVERLAY + '/compressed', 10)
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
