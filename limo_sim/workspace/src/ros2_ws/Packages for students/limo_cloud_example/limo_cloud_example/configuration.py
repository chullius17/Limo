"""Read only local cloud-node configuration; no image-producing nodes or profiles."""

from pathlib import Path

import yaml


def flatten(mapping, prefix=''):
    result = {}
    for name, value in mapping.items():
        key = prefix + name
        if isinstance(value, dict):
            result.update(flatten(value, key + '.'))
        else:
            result[key] = value
    return result


def load_parameters(filename):
    with open(Path(filename).expanduser(), encoding='utf-8') as stream:
        content = yaml.safe_load(stream)
    if not isinstance(content, dict):
        raise ValueError('Expected a ROS parameter mapping')
    parameters = content.get('limo_cloud_example', {}).get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ValueError('Expected limo_cloud_example.ros__parameters mapping')
    return flatten(parameters)
