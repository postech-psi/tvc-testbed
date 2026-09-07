"""
The ROS 2 pipeline with the ANALYTIC plant. No Gazebo, no physics engine.

    ros2 launch tvc_control analytic.launch.py

Identical to gazebo.launch.py except for which plant process runs: same
controller node, same ActuatorCommand, same odometry topic, same gains. That
symmetry is deliberate and it is the whole point of the layer split -- "run the
same scenario on both plants and compare" is a choice of launch file rather than
a porting exercise.

Use it to attribute a failure. If a manoeuvre misbehaves here AND in Gazebo, the
control code is responsible; if only in Gazebo, the difference is in the physics
engine or the SDF. Also the fast inner loop while tuning, since it starts in
under a second.

Arguments:
    gain_profile    profile from control_gains.yaml (empty = the file default)
    altitude_hold   close the altitude loop            (default true)
    position_hold   close the horizontal position loop (default false)
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
            description='Profile from control_gains.yaml; empty uses the '
                        "file's default_profile."),
        DeclareLaunchArgument('altitude_hold', default_value='true'),
        DeclareLaunchArgument('position_hold', default_value='false'),
        DeclareLaunchArgument('att_pitch_des_deg', default_value='0.0'),
        DeclareLaunchArgument('att_yaw_des_deg', default_value='0.0'),
        DeclareLaunchArgument('att_roll_des_deg', default_value='0.0'),

        Node(
            package='tvc_control', executable='simulator_node',
            name='simulator_node', output='screen',
            parameters=[{
                # 250 Hz to match Gazebo's odometry publisher, so the control
                # loop sees the same sample rate on both plants. Changing it
                # here without changing the SDF makes the comparison invalid.
                'dt': 0.004,
                'init_att_pitch_deg': 3.0,
                'init_att_yaw_deg': -4.0,
                'init_z': 2.0,
                # There is no gimbal_rate_max_deg parameter. The gimbal has two
                # rings with different measured slew rates (403 inner, 235
                # outer); a single scalar cannot express that, and the node
                # rejects the name rather than ignoring it.
            }]),

        Node(
            package='tvc_control', executable='controller_node',
            name='controller_node', output='screen',
            parameters=[{
                'rate_hz': 250.0,
                'gain_profile': profile,
                'att_pitch_des_deg': LaunchConfiguration('att_pitch_des_deg'),
                'att_yaw_des_deg': LaunchConfiguration('att_yaw_des_deg'),
                'att_roll_des_deg': LaunchConfiguration('att_roll_des_deg'),
                'altitude_hold': LaunchConfiguration('altitude_hold'),
                'position_hold': LaunchConfiguration('position_hold'),
                'z_des': 2.0,
            }]),
    ])
