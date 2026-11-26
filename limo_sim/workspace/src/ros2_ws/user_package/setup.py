from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'user_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.xml'))),
        ('share/' + package_name + '/scripts', glob(os.path.join('scripts', '*.sh'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ] + [
        ('share/' + package_name + '/launch/' + group,
            glob(os.path.join('launch', group, '*.launch.py')))
        for group in ('amcl', 'ekf', 'control')
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
            'control_experiment = user_package.control.control_experiment:main',
            'amcl_visualization = user_package.amcl.amcl_visualization:main',
            'amcl_stats = user_package.amcl.amcl_stats:main',
            'circuit_experiment = user_package.control.circuit_experiment:main',
            'ekf_experiment = user_package.ekf.ekf_experiment:main',
        ],
    },
)
