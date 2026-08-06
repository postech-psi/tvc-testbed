from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tvc_demo',
            executable='counter_publisher',
            name='counter_publisher',
        ),
        Node(
            package='tvc_demo',
            executable='counter_subscriber',
            name='counter_subscriber',
        ),
    ])
