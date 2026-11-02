from setuptools import setup

setup(
    name='limo_runtime',
    version='0.0.0',
    packages=['limo_runtime'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/limo_runtime']),
        ('share/limo_runtime', ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Giulio Cataldo',
    maintainer_email='giulio.cataldo@studio.unibo.it',
    description='Shared low-overhead ROS Python runtime helpers.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
