"""Start one Gazebo/ROS simulation, using the current settings and final CAD visual."""
import os
from pathlib import Path
import runpy
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    assets = Path(get_package_share_directory("tvc_control")) / "gazebo"
    runtime = tempfile.TemporaryDirectory(prefix="tvc-gazebo-")
    model_dir = Path(runtime.name) / "tvc_vehicle"
    model_dir.mkdir()
    mesh = assets / "models/tvc_vehicle/meshes/tvc_vehicle.stl"
    generator = runpy.run_path(str(assets / "generate_model.py"))
    (model_dir / "model.sdf").write_text(generator["model_xml"](mesh.as_uri()), encoding="utf-8")
    shutil.copyfile(assets / "models/tvc_vehicle/model.config", model_dir / "model.config")
    world = str(assets / "worlds/tvc_flight.sdf")

    def cleanup(event, context):
        runtime.cleanup()

    resource_path = runtime.name
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_path += os.pathsep + os.environ["GZ_SIM_RESOURCE_PATH"]
    gui = LaunchConfiguration("gui")
    args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("z_des", default_value="2.0"),
        DeclareLaunchArgument("log_path", default_value=str(Path.cwd() / "out/gazebo.csv")),
    ]
    gz_gui = ExecuteProcess(cmd=["gz", "sim", world], output="screen", condition=IfCondition(gui))
    gz_server = ExecuteProcess(cmd=["gz", "sim", "-s", "--headless-rendering", world],
                               output="screen", condition=UnlessCondition(gui))
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/tvc_vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tvc_vehicle/gimbal_inner_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/tvc_vehicle/gimbal_outer_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/tvc_vehicle/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
        ], parameters=[{"use_sim_time": True}], output="screen")
    adapter = Node(package="tvc_control", executable="gazebo_bridge_node",
                   parameters=[{"use_sim_time": True}], output="screen")
    controller = Node(
        package="tvc_control", executable="controller_node", output="screen",
        parameters=[{"use_sim_time": True, "rate_hz": 250.0,
                     "altitude_hold": True, "position_hold": True,
                     "z_des": LaunchConfiguration("z_des"),
                     "log_path": LaunchConfiguration("log_path")}])
    # Start paused so the controller can connect before the airborne vehicle moves.
    unpause = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=["gz", "service", "-s", "/world/tvc_flight/control", "--reqtype",
             "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout",
             "3000", "--req", "pause: false"], output="screen")])
    return LaunchDescription(args + [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        gz_gui, gz_server, bridge, adapter, controller, unpause,
    ])
