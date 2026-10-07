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

"""Connect Foxy's raw RViz Image display to the compressed waterfall overlay."""

import yaml


WATERFALL_OVERLAY = '/limo/cv_package/detection/lane_waterfall_overlay'


def waterfall_overlay_bridge(rviz_config, viewer):
    """Return a desktop decoder specification and the RViz topic remapping."""
    with open(rviz_config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    displays = config.get('Visualization Manager', {}).get('Displays', [])
    for display in displays:
        topic = display.get('Topic', {})
        topic = topic.get('Value', '') if isinstance(topic, dict) else topic
        if (display.get('Class') == 'rviz_default_plugins/Image'
                and display.get('Enabled', False) and topic == WATERFALL_OVERLAY):
            output_topic = '/limo/desktop/{}/lane_waterfall_overlay'.format(viewer)
            return {
                'package': 'image_transport',
                'executable': 'republish',
                'name': viewer + '_waterfall_overlay_decoder',
                'output': 'screen',
                'arguments': ['compressed', 'raw'],
                'remappings': [
                    ('in/compressed', WATERFALL_OVERLAY + '/compressed'),
                    ('out', output_topic),
                ],
            }, [(WATERFALL_OVERLAY, output_topic)]
    return None, []
