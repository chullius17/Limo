"""Run the real CV backend with the C++ projector and Python semantic plugins."""

from limo_cloud_example.cv_launch import generate_launch_description as cv_description


def generate_launch_description():
    return cv_description(profile='real', mode='backend')
