"""
The full software-in-the-loop stack: gz-sim plant + ROS 2 + the flight code.

    ros2 launch tvc_control gazebo.launch.py            # with the Gazebo window
    ros2 launch tvc_control gazebo.launch.py gui:=false # headless (Windows/macOS)

Five processes:
    gz sim                world + vehicle SDF, the physics
    ros_gz_bridge         gz topics <-> ROS topics, including /clock
    gazebo_bridge_node    ActuatorCommand -> the three gz plugin topics
    controller_node       odometry -> TvcController -> ActuatorCommand
    (a timer)             unpauses the world once the nodes have discovered
                          each other

FOUR THINGS THAT LOOK LIKE CONTROL BUGS AND ARE NOT, EACH HANDLED HERE

  STARTING UNPAUSED. `gz sim -r` integrates immediately and the vehicle spawns
  airborne at 2 m, so it free-falls in ~0.6 s -- less time than ROS 2 node
  discovery takes. A controller connecting afterwards finds it on the ground.
  This starts paused and unpauses on a TimerAction.

  NOT BRIDGING /clock. Without it ROS time is wall-clock while Gazebo runs on
  simulated time, so every measured dt is wrong by whatever the real-time factor
  happens to be -- and that varies with machine load.

  A `gui` ARGUMENT THAT NOTHING READS. Headless is mandatory on Windows and
  macOS hosts, where the container has no display.

  GAINS INLINED HERE. They were an independent third copy. Gains come from
  control_gains.yaml by profile name and from nowhere else.

Arguments:
    world           SDF to load (default: gazebo/worlds/tvc_flight.sdf)
    gui             false runs the server only
    gain_profile    profile from control_gains.yaml (empty = the file default)
    altitude_hold / position_hold / z_des
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
    # install/tvc_control/share/tvc_control -> repo root. The SDF assets are not
    # installed into the share directory because gz-sim, run_hover.sh and this
    # file all want the same copy, and two copies of a world is one too many.
    repo_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    models = os.path.join(repo_root, 'gazebo', 'models')
    default_world = os.path.join(repo_root, 'gazebo', 'worlds', 'tvc_flight.sdf')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    profile = LaunchConfiguration('gain_profile')

    args = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='World SDF. Defaults to tvc_flight.sdf, which spawns '
                        'airborne and deliberately tilted -- an upright spawn '
                        'sits in equilibrium with the gimbal at exactly zero '
                        'and proves nothing.'),
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
            '/tvc_vehicle/gimbal_inner_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/tvc_vehicle/gimbal_outer_cmd@std_msgs/msg/Float64]gz.msgs.Double',
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
            'att_pitch_des_deg': 0.0,
            'att_yaw_des_deg': 0.0,
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
