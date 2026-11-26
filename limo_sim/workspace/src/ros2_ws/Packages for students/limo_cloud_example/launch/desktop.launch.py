"""Open the original CV viewer on the desktop; the backend runs on the robot."""

from limo_cloud_example.cv_launch import generate_launch_description as cv_description


def generate_launch_description():
    return cv_description(mode='desktop', start_rviz='true')
