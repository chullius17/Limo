import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'limo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            'share/' + package_name + '/config',
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gui = limo_controller.control_gui:main',
            'cmd_vel_mux = limo_controller.cmd_vel_mux:main',
            'path_executor = limo_controller.path_executor:main',
            'mpc_preview = limo_controller.mpc_preview.node:main',
            'mpc_gui = limo_controller.mpc_preview.gui:main',
        ],
    },
)
