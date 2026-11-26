"""Run the simulated CV pipeline and its usual RViz with the new cloud stage."""

from limo_cloud_utilities.cv_launch import generate_launch_description as cv_description


def generate_launch_description():
    return cv_description(profile='sim', start_rviz='true')
