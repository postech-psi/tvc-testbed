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
    ],
    # The single source of truth for every vehicle number, installed NEXT TO THE
    # MODULE because that is where config.py looks for it -- one directory up
    # from gnc/, exactly as in the source tree, so an installed node and a
    # source-tree run read the same path expression.
    #
    # These used to be listed in data_files as share/tvc_control/*.yaml. That
    # put them somewhere nothing reads: every colcon-installed node died on
    # startup with "vehicle_params.yaml not found at .../site-packages/
    # tvc_control/vehicle_params.yaml". The packages built, the entry points
    # installed, the modules imported -- and no node could start. Only
    # `ros2 launch` finds that, which is why it is now in CI's sights.
    package_data={package_name: ['*.yaml']},
    install_requires=['setuptools'],
    # False because package_data is real files that must exist on disk next to
    # the module; a zipped egg would hide them from config.py's open().
    zip_safe=False,
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
