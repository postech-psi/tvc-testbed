"""Install the Python package, ROS launch file, settings and Gazebo assets."""
from glob import glob
from setuptools import find_packages, setup

setup(
    name="tvc_control",
    version="0.0.1",
    packages=find_packages(),
    package_data={"tvc_control": ["settings/*.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tvc_control"]),
        ("share/tvc_control", ["package.xml"]),
        ("share/tvc_control/launch", glob("launch/*.launch.py")),
        ("share/tvc_control/gazebo", ["gazebo/generate_model.py"]),
        ("share/tvc_control/gazebo/worlds", glob("gazebo/worlds/*.sdf")),
        ("share/tvc_control/gazebo/models/tvc_vehicle", ["gazebo/models/tvc_vehicle/model.config"]),
        ("share/tvc_control/gazebo/models/tvc_vehicle/meshes", glob("gazebo/models/tvc_vehicle/meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="devkyber",
    maintainer_email="kyber06@icloud.com",
    description="TVC simulation and shared controller for the POSTECH UGRP demonstrator",
    license="TODO: License declaration",
    entry_points={"console_scripts": [
        "controller_node = tvc_control.ros.controller:main",
        "gazebo_bridge_node = tvc_control.ros.gazebo_bridge:main",
        "tvc = tvc_control.__main__:main",
    ]},
)
