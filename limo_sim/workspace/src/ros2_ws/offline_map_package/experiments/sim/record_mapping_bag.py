#!/usr/bin/env python3
"""Record original-rate mapper inputs, with configuration/reference snapshots.

Run in the ROS-sourced simulation container. Stop with Ctrl+C. The bag contains
the original clock, TF, SLAM map, semantic cloud and scans so replay can keep
SLAM and vision fixed while varying only the semantic mapper parameters.
"""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

import yaml


DIRECTORY = Path(__file__).resolve().parent
PACKAGE = DIRECTORY.parents[1]
SOURCE = DIRECTORY.parents[3]


def recording_spec(profile):
    """Derive input topics and QoS from the selected mapping profile."""
    mapper = profile['semantic_mapper']
    cloud = mapper['cloud_topic']
    reference_map = mapper['reference_map_topic']
    scan = profile['slam_toolbox']['scan_topic']
    required = list(dict.fromkeys([cloud, scan, '/tf', '/tf_static', '/clock', reference_map]))
    topics = list(dict.fromkeys(required + ['/odom', '/odometry/filtered']))
    sensor = {'reliability': 'best_effort', 'durability': 'volatile',
              'history': 'keep_last', 'depth': 100}
    latched = {'reliability': 'reliable', 'durability': 'transient_local',
               'history': 'keep_last', 'depth': 100}
    reliable = {'reliability': 'reliable', 'durability': 'volatile',
                'history': 'keep_last', 'depth': 100}
    qos = {cloud: sensor, scan: sensor, '/tf': reliable,
           '/tf_static': latched, reference_map: latched,
           '/clock': sensor, '/odom': reliable, '/odometry/filtered': reliable}
    return required, topics, qos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', help='Unique session directory name')
    parser.add_argument('--mapping-config', type=Path,
                        default=PACKAGE / 'config/mapping_sim.yaml')
    parser.add_argument('--output-root', type=Path, default=DIRECTORY / 'bags')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print recording settings without creating files or starting ROS')
    args = parser.parse_args()
    with args.mapping_config.open(encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    required, topics, qos = recording_spec(profile)
    name = args.name or datetime.now(timezone.utc).strftime('run_%Y%m%d_%H%M%SZ')
    if not name or Path(name).name != name or name in ('.', '..'):
        parser.error('--name must be a directory name without a path')
    session = args.output_root.resolve() / name
    command = ['ros2', 'bag', 'record', '-o', str(session / 'bag'),
               '--qos-profile-overrides-path', str(session / 'qos.yaml')] + topics
    print('Required topics: {}'.format(', '.join(required)))
    print('Recording command: {}'.format(' '.join(shlex.quote(value) for value in command)),
          flush=True)
    if args.dry_run:
        print(yaml.safe_dump(qos, sort_keys=False))
        return 0
    if shutil.which('ros2') is None:
        parser.error('Source the ROS Foxy environment before recording')
    try:
        discovered = subprocess.run(['ros2', 'topic', 'list'], check=True,
                                    capture_output=True, text=True, timeout=15)
    except (subprocess.SubprocessError, OSError) as error:
        parser.error('Could not discover ROS topics: {}'.format(error))
    missing = sorted(set(required) - set(discovered.stdout.splitlines()))
    if missing:
        parser.error('Start Gazebo, SLAM and CV first; missing topics: ' + ', '.join(missing))
    if session.exists():
        parser.error('Session already exists; choose a new --name: {}'.format(session))

    cv_config = Path(profile['launch'].get('cv_config', 'cv_sim.yaml'))
    if not cv_config.is_absolute():
        cv_config = SOURCE / 'ros2_ws/cv_package/config' / cv_config
    snapshots = {
        'mapping_profile.yaml': args.mapping_config,
        'cv_profile.yaml': cv_config,
        'gazebo_model.sdf': SOURCE / 'custom_start/models/plane_test/model.sdf',
        'gazebo_world.world': SOURCE / 'custom_start/worlds/limo_circuit_world.world',
        'ideal_map.pgm': DIRECTORY / 'ideal_map/ideal_map.pgm',
        'ideal_map.yaml': DIRECTORY / 'ideal_map/ideal_map.yaml',
    }
    absent = [str(path) for path in snapshots.values() if not path.is_file()]
    if absent:
        parser.error('Missing experiment inputs: ' + ', '.join(absent))
    session.mkdir(parents=True)
    for target, source in snapshots.items():
        shutil.copyfile(str(source), str(session / target))
    (session / 'qos.yaml').write_text(yaml.safe_dump(qos, sort_keys=False), encoding='utf-8')
    manifest = {'topics': topics, 'required_topics': required,
                'recorded_clock': '/clock', 'playback_rate': 1.0,
                'reference_frame': 'gazebo_world',
                'input_profile': str(args.mapping_config.resolve()),
                'command': command}
    (session / 'session.yaml').write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')

    # Capture effective node settings when present, in addition to source profiles.
    for node in ('semantic_mapper', 'slam_toolbox', 'visual_ptcld'):
        try:
            result = subprocess.run(['ros2', 'param', 'dump', '/' + node, '--print'],
                                    capture_output=True, text=True, timeout=5, cwd=str(session))
            parameters = yaml.safe_load(result.stdout) if result.returncode == 0 else None
            if isinstance(parameters, dict) and any(
                    str(key).lstrip('/') == node for key in parameters):
                (session / (node + '_parameters.yaml')).write_text(
                    result.stdout, encoding='utf-8')
        except (subprocess.SubprocessError, OSError, yaml.YAMLError):
            pass
    print('Session: {}\nStop with Ctrl+C. Recording every received message.'.format(session),
          flush=True)
    process = subprocess.Popen(command)
    try:
        return process.wait()
    except KeyboardInterrupt:
        # The terminal delivers Ctrl+C to rosbag as well; wait for metadata to flush.
        return process.wait()


if __name__ == '__main__':
    sys.exit(main())
