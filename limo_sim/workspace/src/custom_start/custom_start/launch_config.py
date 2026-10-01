"""Read launch settings from a YAML profile with optional CLI overrides."""

import os

import yaml
from launch.substitutions import LaunchConfiguration


def load_profile(context, sections):
    """Load the YAML file selected by the config_file launch argument."""
    path = os.path.expanduser(
        LaunchConfiguration('config_file').perform(context))
    with open(path, encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    for section in sections:
        if not isinstance(profile, dict) or not isinstance(
                profile.get(section), dict):
            raise ValueError(
                '{}: missing YAML mapping {!r}'.format(path, section))
    return profile


def setting(context, settings, name):
    """Return a launch-friendly scalar, giving the CLI value precedence."""
    override = LaunchConfiguration(name).perform(context)
    value = override if override != '' else settings[name]
    if isinstance(value, bool):
        return str(value).lower()
    if value is None or not isinstance(value, (str, int, float)):
        raise ValueError('{} must be a string, number or boolean'.format(name))
    return str(value)


def scalar(settings, name):
    """Convert a profile-only scalar to the string expected by launch."""
    value = settings[name]
    if isinstance(value, bool):
        return str(value).lower()
    if value is None or not isinstance(value, (str, int, float)):
        raise ValueError('{} must be a string, number or boolean'.format(name))
    return str(value)


def boolean(value, name):
    """Parse a launch boolean and reject typos in YAML or CLI arguments."""
    if value.lower() not in ('true', 'false'):
        raise ValueError('{} must be true or false'.format(name))
    return value.lower() == 'true'
