"""Check complete-stack wiring, profile selection and existing-input overrides."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[1]
EXTENSIONS = ROOT.parent
PACKAGES = EXTENSIONS.parent


def package_share(name):
    return str(EXTENSIONS / name if (EXTENSIONS / name).is_dir() else PACKAGES / name)


class MappingLaunch(unittest.TestCase):
    def launch(self, **overrides):
        spec = importlib.util.spec_from_file_location('mapping_example', ROOT / 'launch/map.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        with patch.object(module, 'get_package_share_directory', side_effect=package_share), \
                patch.object(module, 'Node', side_effect=lambda **kwargs: {'node': kwargs}), \
                patch.object(module, 'GroupAction', side_effect=lambda **kwargs: {'group': kwargs['actions']}), \
                patch.object(module, 'PythonLaunchDescriptionSource', side_effect=lambda path: path), \
                patch.object(module, 'IncludeLaunchDescription', side_effect=lambda source, **kwargs: {
                    'source': source, 'arguments': dict(kwargs['launch_arguments'])}):
            return module._launch_mapping(context)

    def test_simulation_default_is_complete_and_has_one_viewer(self):
        actions = self.launch()
        cv = actions[0]['group'][0]
        self.assertTrue(cv['source'].endswith('limo_cloud_example/launch/sim.launch.py'))
        self.assertEqual(cv['arguments']['mode'], 'backend')
        self.assertEqual(cv['arguments']['start_rviz'], 'false')
        self.assertEqual(cv['arguments']['use_sim_time'], 'true')
        nodes = [action['node'] for action in actions if 'node' in action]
        self.assertEqual([node['package'] for node in nodes],
                         ['slam_toolbox', 'limo_map', 'image_transport', 'rviz2'])
        self.assertIn('slam_toolbox:use_sim_time:=true', nodes[0]['arguments'])
        self.assertTrue(nodes[0]['parameters'][0].endswith('config/slam.yaml'))
        self.assertIn('limo_map:use_sim_time:=true', nodes[1]['arguments'])
        gui = actions[3]['group'][0]
        self.assertTrue(gui['source'].endswith('map_save_gui.launch.py'))
        self.assertTrue(gui['arguments']['config_file'].endswith('config/gui.yaml'))
        self.assertTrue(nodes[1]['parameters'][0].endswith('config/map.yaml'))

    def test_real_profile_changes_cv_mapper_viewer_and_clock(self):
        actions = self.launch(profile='real')
        self.assertTrue(actions[0]['group'][0]['source'].endswith('real.launch.py'))
        self.assertEqual(actions[0]['group'][0]['arguments']['use_sim_time'], 'false')
        nodes = [action['node'] for action in actions if 'node' in action]
        self.assertIn('slam_toolbox:use_sim_time:=false', nodes[0]['arguments'])
        self.assertTrue(nodes[0]['parameters'][0].endswith('config/slam_real.yaml'))
        self.assertTrue(nodes[1]['parameters'][0].endswith('config/map_real.yaml'))
        self.assertTrue(nodes[-1]['arguments'][1].endswith('offline_map_real.rviz'))

    def test_only_mapper_can_consume_existing_inputs(self):
        actions = self.launch(start_cv='false', start_slam='false', start_gui='false', start_rviz='false')
        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0]['node']['executable'], 'limo_map_node')

    def test_slam_tuning_matches_the_original_offline_profiles(self):
        for profile, filename in [('sim', 'slam.yaml'), ('real', 'slam_real.yaml')]:
            with self.subTest(profile=profile):
                original = yaml.safe_load((PACKAGES / 'offline_map_package/config' /
                                           ('mapping_' + profile + '.yaml')).read_text())['slam_toolbox']
                # The previous simulation launch did not override these native
                # SLAM defaults; make them explicit to catch accidental mixing.
                expected = {'minimum_travel_distance': 0.5, 'minimum_travel_heading': 0.5}
                expected.update(original)
                actual = yaml.safe_load((ROOT / 'config' / filename).read_text())[
                    'slam_toolbox']['ros__parameters']
                for name, setting in expected.items():
                    self.assertEqual(actual[name], setting, profile + ':' + name)

    def test_custom_mapper_settings_reach_slam_and_gui(self):
        with tempfile.NamedTemporaryFile(mode='w+', suffix='.yaml') as config:
            config.write('''limo_map:
  ros__parameters:
    map_frame: world
    odom_frame: wheel_odom
    resolution: 0.1
    reference_map_topic: /laser_grid
    output_prefix: /custom_mapping
''')
            config.flush()
            actions = self.launch(config_file=config.name, start_rviz='false', use_sim_time='false')
        slam = actions[1]['node']
        self.assertIn('slam_toolbox:map_frame:=world', slam['arguments'])
        self.assertIn('slam_toolbox:odom_frame:=wheel_odom', slam['arguments'])
        self.assertIn('slam_toolbox:resolution:=0.1', slam['arguments'])
        self.assertEqual(slam['remappings'], [('map', '/laser_grid')])
        gui = actions[3]['group'][0]['arguments']
        self.assertEqual(gui['save_service'], '/custom_mapping/map_saver/save_map')
        self.assertEqual(gui['status_topic'], '/custom_mapping/map_saver/status')
        self.assertEqual(gui['use_sim_time'], 'false')

    def test_cartographer_requires_external_slam(self):
        with tempfile.NamedTemporaryFile(mode='w+', suffix='.yaml') as config:
            config.write('limo_map:\n  ros__parameters:\n    pose_source: cartographer\n')
            config.flush()
            with self.assertRaisesRegex(ValueError, 'start_slam:=false'):
                self.launch(config_file=config.name)
            actions = self.launch(config_file=config.name, start_slam='false', start_rviz='false')
            self.assertFalse(any(action.get('node', {}).get('package') == 'slam_toolbox'
                                 for action in actions))


if __name__ == '__main__':
    unittest.main()
