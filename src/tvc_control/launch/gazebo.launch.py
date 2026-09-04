"""
gazebo.launch.py -- the software-in-the-loop stack: gz-sim plant + flight code.

Four things this launch file used to get wrong, each of which would have made
the first run look like a control bug:

  IT STARTED UNPAUSED. `gz sim -r` begins integrating immediately, and the
  vehicle spawns airborne at 2 m in tvc_flight.sdf. It free-falls in ~0.6 s,
  which is less time than ROS 2 node discovery takes, so a controller that
  connected afterwards would find the vehicle already on the ground.
  sim/run_hover.sh documents this and starts paused; so does this file now.

  IT NEVER BRIDGED /clock. Without it, ROS time is wall-clock while Gazebo runs
  on sim time, so every measured dt is wrong by whatever the real-time factor
  happens to be -- and varies with machine load.

  IT DECLARED A `gui` ARGUMENT AND IGNORED IT. sim/README.md told people to pass
  gui:=false for headless; nothing read it.

  IT INLINED THE GAINS. Eight gain parameters were duplicated here, a third copy
  after physics.py's defaults and hover.py's constants. Gains now come from
  control_gains.yaml by profile name.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tvc_control')
    # install/tvc_control/share/tvc_control -> repo root
    repo_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    models = os.path.join(repo_root, 'sim', 'models')
    default_world = os.path.join(repo_root, 'sim', 'worlds', 'tvc_flight.sdf')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    profile = LaunchConfiguration('gain_profile')

    args = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='World SDF. Defaults to tvc_flight.sdf, which spawns '
                        'airborne and deliberately tilted -- an upright spawn '
                        'on the ground tests nothing until takeoff exists.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false runs gz sim headless (server only).'),
        DeclareLaunchArgument(
            'gain_profile', default_value='',
            description='Profile name from control_gains.yaml; empty uses the '
                        'file default_profile.'),
        DeclareLaunchArgument('altitude_hold', default_value='true'),
        DeclareLaunchArgument('position_hold', default_value='true'),
        DeclareLaunchArgument('z_des', default_value='2.0'),
    ]

    # Append rather than overwrite: a user may already have models on the path,
    # and silently dropping them is a confusing failure mode.
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models if not existing else models + os.pathsep + existing

    # Started PAUSED (no -r). The TimerAction below unpauses once the nodes are
    # up. See the header for why this is not optional.
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', world],
        output='screen', condition=IfCondition(gui))
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '--headless-rendering', world],
        output='screen', condition=UnlessCondition(gui))

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        arguments=[
            # '[' is gz->ROS, ']' is ROS->gz.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/tvc_vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tvc_vehicle/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tvc_vehicle/gimbal_pitch@std_msgs/msg/Float64]gz.msgs.Double',
            '/tvc_vehicle/gimbal_roll@std_msgs/msg/Float64]gz.msgs.Double',
            '/tvc_vehicle/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    common = {'use_sim_time': True}

    hal = Node(package='tvc_control', executable='gazebo_bridge_node',
               name='gazebo_bridge_node', parameters=[common], output='screen')

    controller = Node(
        package='tvc_control', executable='controller_node',
        name='controller_node', output='screen',
        parameters=[{
            **common,
            'rate_hz': 250.0,          # matches the odometry publisher
            'gain_profile': profile,
            'roll_des_deg': 0.0,
            'pitch_des_deg': 0.0,
            'altitude_hold': LaunchConfiguration('altitude_hold'),
            'position_hold': LaunchConfiguration('position_hold'),
            'z_des': LaunchConfiguration('z_des'),
        }])

    # Unpause only after the nodes have had time to discover each other. The
    # world name is derived from the SDF filename, which is how both worlds in
    # this repo are named.
    world_name = PythonExpression(
        ["'", world, "'.split('/')[-1].split('\\\\')[-1].rsplit('.', 1)[0]"])
    unpause = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['gz', 'service', '-s',
             PythonExpression(["'/world/' + '", world_name, "' + '/control'"]),
             '--reqtype', 'gz.msgs.WorldControl',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', 'pause: false'],
        output='screen')])

    return LaunchDescription(
        args + [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
                gz_gui, gz_headless, bridge, hal, controller, unpause])
