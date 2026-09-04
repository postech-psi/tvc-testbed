from setuptools import find_packages, setup
from glob import glob

package_name = 'tvc_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # The single source of truth for every vehicle number. Installed so a
        # colcon-installed node can find it without reaching back into the
        # source tree -- the alternative was literal fallback constants, and
        # those went stale (20.0 N thrust survived in three files after the
        # bench measured 17.79 N).
        ('share/' + package_name, ['tvc_control/vehicle_params.yaml',
                                  'tvc_control/control_gains.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devkyber',
    maintainer_email='kyber06@icloud.com',
    description='TVC VTVL flight code, plant models and ROS 2 nodes for the '
                'POSTECH UGRP coaxial thrust-vectored demonstrator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = tvc_control.nodes.simulator:main',
            'controller_node = tvc_control.nodes.controller:main',
            'gazebo_bridge_node = tvc_control.nodes.gazebo_bridge:main',
            # The same CLI as ./tvc.py, available once the package is installed.
            'tvc = tvc_control.__main__:main',
        ],
    },
)
