"""Complete launch wiring, native pipeline parity and independent Nav2 overlay."""
import importlib.util
import re
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


class AmclLaunch(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        for suffix in ['complete', 'laser', 'cv_obstacle']:
            (Path(self.directory.name) / ('limo_map_' + suffix + '.yaml')).touch()

    def launch(self, **overrides):
        spec = importlib.util.spec_from_file_location('amcl_example', ROOT / 'launch/amcl.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update({'map_directory': self.directory.name, **overrides})
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        with patch.object(module, 'get_package_share_directory', side_effect=package_share), \
                patch.object(module, 'Node', side_effect=lambda **kwargs: {'node': kwargs}), \
                patch.object(module, 'GroupAction', side_effect=lambda **kwargs: {'group': kwargs['actions']}), \
                patch.object(module, 'PythonLaunchDescriptionSource', side_effect=lambda path: path), \
                patch.object(module, 'IncludeLaunchDescription', side_effect=lambda source, **kwargs: {
                    'source': source, 'arguments': dict(kwargs['launch_arguments'])}):
            return module._launch(context)

    def test_complete_example_has_only_own_amcl_and_one_viewer(self):
        actions = self.launch()
        nodes = [a['node'] for a in actions if 'node' in a]
        self.assertEqual(sum(n['executable'] == 'map_server' for n in nodes), 3)
        self.assertEqual(sum(n['package'] == 'rviz2' for n in nodes), 1)
        self.assertFalse(any(n['package'] in ('nav2_amcl', 'slam_toolbox') for n in nodes))
        amcl = next(n for n in nodes if n['package'] == 'limo_amcl')
        self.assertEqual(amcl['executable'], 'limo_amcl_node')
        self.assertEqual(amcl['parameters'][0]['laser_weight_factor'], 2.0)
        self.assertEqual(amcl['parameters'][0]['cv_road_class_ids'], [1, 5])
        self.assertTrue(amcl['parameters'][0]['use_sim_time'])
        manager = next(n for n in nodes if n['package'] == 'nav2_lifecycle_manager')
        self.assertEqual(manager['parameters'][0]['node_names'], [
            'limo_amcl_complete_map_server', 'limo_amcl_laser_map_server',
            'limo_amcl_cv_map_server', 'amcl'])
        cv = next(a['group'][0] for a in actions if 'group' in a)
        self.assertTrue(cv['source'].endswith('limo_cloud_utilities/launch/sim.launch.py'))
        self.assertEqual(cv['arguments']['mode'], 'backend')
        self.assertEqual(cv['arguments']['start_rviz'], 'false')

    def test_profiles_preserve_all_previous_amcl_tuning(self):
        for profile, filename in [('sim', 'amcl.yaml'), ('real', 'amcl_real.yaml')]:
            previous = yaml.safe_load((PACKAGES / 'online_map_package/config' /
                                       ('mapping_' + profile + '.yaml')).read_text())['amcl']
            actual = yaml.safe_load((ROOT / 'config' / filename).read_text())['amcl']['ros__parameters']
            # Include commented native defaults to check settings deliberately left implicit.
            defaults = yaml.safe_load(re.sub(r'(?m)^    # ([a-z][a-z0-9_.]*: )',
                                            r'    \1', (ROOT / 'config' / filename).read_text()))
            actual = {**defaults['amcl']['ros__parameters'], **actual}
            for name, setting in previous.items():
                self.assertEqual(actual[name], setting, profile + ':' + name)

    def test_real_profile_and_clock_override_are_independent(self):
        actions = self.launch(profile='real', use_sim_time='true')
        cv = next(a['group'][0] for a in actions if 'group' in a)
        self.assertTrue(cv['source'].endswith('real.launch.py'))
        self.assertEqual(cv['arguments']['use_sim_time'], 'true')
        amcl = next(a['node'] for a in actions if a.get('node', {}).get('package') == 'limo_amcl')
        self.assertEqual(amcl['parameters'][0]['laser_weight_factor'], 1.0)
        rviz = next(a['node'] for a in actions if a.get('node', {}).get('package') == 'rviz2')
        self.assertTrue(rviz['arguments'][1].endswith('amcl_real.rviz'))

    def test_existing_nav2_map_and_cv_can_be_reused(self):
        actions = self.launch(start_maps='false', start_cv='false', start_rviz='false',
                              map_directory='/no/maps/needed', map_topic='/map', cv_enabled='false')
        self.assertEqual([a['node']['package'] for a in actions], ['limo_amcl', 'nav2_lifecycle_manager'])
        params = actions[0]['node']['parameters'][0]
        self.assertFalse(params['cv_enabled'])
        self.assertEqual(params['map_topic'].evaluate(LaunchContext()), '/map')
        self.assertEqual(actions[1]['node']['parameters'][0]['node_names'], ['amcl'])

    def test_existing_nav2_lifecycle_manager_can_own_amcl(self):
        actions = self.launch(start_maps='false', start_cv='false', start_rviz='false',
                              start_lifecycle_manager='false')
        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0]['node']['package'], 'limo_amcl')

    def test_custom_topics_reach_both_map_servers_and_amcl(self):
        actions = self.launch(map_topic='/custom/laser', cv_map_topic='/custom/cv',
                              complete_map_topic='/custom/map', start_rviz='false')
        nodes = [a['node'] for a in actions if 'node' in a]
        self.assertEqual([n['remappings'][0][1] for n in nodes[:3]],
                         ['/custom/map', '/custom/laser', '/custom/cv'])
        with self.assertRaisesRegex(ValueError, 'distinct topics'):
            self.launch(map_topic='/map')

    def test_missing_maps_fail_before_starting_nodes(self):
        with self.assertRaisesRegex(ValueError, 'Save maps with limo_map'):
            self.launch(map_name='missing')
        with self.assertRaisesRegex(ValueError, 'filename stem'):
            self.launch(map_name='../unsafe')
        with self.assertRaisesRegex(ValueError, 'profile must be'):
            self.launch(profile='unknown')


if __name__ == '__main__':
    unittest.main()
