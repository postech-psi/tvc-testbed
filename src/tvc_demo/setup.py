from setuptools import find_packages, setup
from glob import glob

package_name = 'tvc_demo'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devkyber',
    maintainer_email='kyber06@icloud.com',
    description='Minimal publisher/subscriber pair to validate the ROS2 dev environment before Phase 4',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'counter_publisher = tvc_demo.counter_publisher:main',
            'counter_subscriber = tvc_demo.counter_subscriber:main',
        ],
    },
)
    