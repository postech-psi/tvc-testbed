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
    description='Phase 4: simulator_node + controller_node, splitting tvc_physics.py across a ROS2 topic boundary',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = tvc_control.simulator_node:main',
            'controller_node = tvc_control.controller_node:main',
            'gazebo_bridge_node = tvc_control.gazebo_bridge_node:main',
        ],
    },
)
