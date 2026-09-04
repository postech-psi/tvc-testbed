from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tvc_control',
            executable='simulator_node',
            name='simulator_node',
            parameters=[{
                'dt': 0.01,
                'init_roll_deg': 3.0,
                'init_pitch_deg': -4.0,
                # gimbal_rate_max_deg was here at 180.0, contradicting the
                # measured per-ring rates (403 inner / 235 outer). It is retired
                # -- simulator_node now rejects it rather than ignoring it.
            }],
        ),
        Node(
            package='tvc_control',
            executable='controller_node',
            name='controller_node',
            parameters=[{
                'dt': 0.01,
                'roll_des_deg': 0.0,
                'pitch_des_deg': 5.0,
                'kp_angle': 4.0,
                'kp_rate': 0.02,
                'ki_rate': 0.002,
                'kd_rate': 0.004,
                'i_limit': 0.5,
            }],
        ),
    ])
