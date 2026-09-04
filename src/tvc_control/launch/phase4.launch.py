"""
phase4.launch.py -- the analytic plant, same flight code, no Gazebo.

Identical to gazebo.launch.py except for which plant process runs. Same
controller node, same ActuatorCommand, same odometry topic. That symmetry is
deliberate: it is what makes "run the same scenario on both plants and compare"
a launch-file argument rather than a porting exercise.

Use it when Gazebo is unavailable, when a failure needs to be attributed to the
control code rather than the physics engine, or as the fast inner loop while
tuning.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration('gain_profile')
    return LaunchDescription([
        DeclareLaunchArgument(
            'gain_profile', default_value='',
            description='Profile from control_gains.yaml; empty uses the file '
                        'default_profile.'),
        DeclareLaunchArgument('altitude_hold', default_value='true'),
        DeclareLaunchArgument('position_hold', default_value='false'),

        Node(
            package='tvc_control', executable='simulator_node',
            name='simulator_node', output='screen',
            parameters=[{
                # 250 Hz to match Gazebo's odometry publisher, so the control
                # loop sees the same sample rate on both plants.
                'dt': 0.004,
                'init_roll_deg': 3.0,
                'init_pitch_deg': -4.0,
                'init_z': 2.0,
                # gimbal_rate_max_deg was passed here at 180.0, contradicting
                # the measured per-ring rates (403 inner / 235 outer). It is
                # retired -- simulator_node rejects it rather than ignoring it.
            }]),

        Node(
            package='tvc_control', executable='controller_node',
            name='controller_node', output='screen',
            parameters=[{
                'rate_hz': 250.0,
                'gain_profile': profile,
                'roll_des_deg': 0.0,
                'pitch_des_deg': 0.0,
                'altitude_hold': LaunchConfiguration('altitude_hold'),
                'position_hold': LaunchConfiguration('position_hold'),
                'z_des': 2.0,
            }]),
    ])
