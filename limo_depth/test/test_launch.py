"""Check launch profile/clock precedence and parameter documentation without a ROS graph."""

import importlib.util
from pathlib import Path
import re
from unittest.mock import patch

from launch import LaunchContext
import yaml


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('depth_launch', ROOT / 'launch/depth.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def resolve(**values):
    context = LaunchContext()
    context.launch_configurations.update(dict(profile='sim', config_file='', use_sim_time=''))
    context.launch_configurations.update(values)
    with patch.object(launch, 'get_package_share_directory', return_value=str(ROOT)), \
            patch.object(launch, 'Node', side_effect=lambda **kwargs: kwargs):
        return launch._launch_depth(context)[0]


if __name__ == '__main__':
    for profile, clock in [('sim', 'true'), ('real', 'false')]:
        node = resolve(profile=profile)
        assert node['package'] == 'limo_depth'
        assert node['executable'] == 'limo_depth_node'
        assert node['parameters'][0]['use_sim_time'] == (clock == 'true')
        assert node['parameters'][0]['calibration_frames'] == (30 if profile == 'sim' else 10)
    node = resolve(profile='real', config_file=str(ROOT / 'config/sim.yaml'), use_sim_time='false')
    assert node['parameters'][0]['calibration_frames'] == 30
    assert node['parameters'][0]['use_sim_time'] is False
    for invalid in [dict(profile='other'), dict(use_sim_time='yes')]:
        try:
            resolve(**invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(invalid)
    source = (ROOT / 'src/depth_node.cpp').read_text()
    declared = set(re.findall(r'(?:parameter(?:<int64_t>)?|integer)\("([a-z_]+)"', source))
    documented = set(re.findall(r'^    # ([a-z_]+):', (ROOT / 'config/depth.yaml').read_text(), re.M))
    assert declared <= documented, 'Undocumented parameters: ' + str(declared - documented)
    for config in (ROOT / 'config').glob('*.yaml'):
        params = yaml.safe_load(config.read_text())['limo_depth']['ros__parameters']
        assert set(params) <= declared | {'use_sim_time'}, config
    print('Launch profiles, clock overrides and parameter documentation OK')
