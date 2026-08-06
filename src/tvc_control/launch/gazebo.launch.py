"""
Phase 5: run the controller against Gazebo instead of simulator_node.

simulator_node.py was written as a swappable seam -- it publishes attitude on
/imu and subscribes to /gimbal_command. This launch file replaces it with
Gazebo + ros_gz_bridge on the same topic names, so controller_node.py runs
unchanged. That is the whole point of the seam: swapping the plant must not
require touching the controller.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tvc_control")
    # sim/ lives at the repo root, not inside the package -- the models are
    # shared with bare `gz sim` runs that don't involve ROS at all.
    repo_root = os.path.abspath(os.path.join(pkg_share, "..", "..", "..", ".."))
    sim_dir = os.path.join(repo_root, "sim")

    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(sim_dir, "worlds", "tvc.sdf"),
            description="SDF world to load",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="false runs headless -- required on Windows/macOS hosts",
        ),

        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "models")),

        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen",
        ),

        # Gazebo <-> ROS2 topic bridge. The ROS-side names deliberately match
        # what controller_node already uses.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            arguments=[
                "/tvc_vehicle/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/tvc_vehicle/gimbal_pitch@std_msgs/msg/Float64]gz.msgs.Double",
                "/tvc_vehicle/gimbal_roll@std_msgs/msg/Float64]gz.msgs.Double",
                "/tvc_vehicle/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
                "/model/tvc_vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            # controller_node subscribes to /sim/vehicle_attitude -- remap
            # Gazebo's IMU onto it so the controller needs no changes.
            remappings=[("/tvc_vehicle/imu", "/sim/vehicle_attitude")],
        ),

        # Translates the controller's GimbalCommand into the two Float64 joint
        # topics Gazebo's JointPositionController expects, and holds the
        # rotors at hover speed. Without this the controller's output has
        # nowhere to go -- Gazebo does not speak tvc_msgs.
        Node(
            package="tvc_control",
            executable="gazebo_bridge_node",
            name="gazebo_bridge_node",
            output="screen",
        ),

        Node(
            package="tvc_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{
                "dt": 0.01,
                "roll_des_deg": 0.0,
                "pitch_des_deg": 0.0,
                "kp_angle": 4.0,
                "kp_rate": 0.02,
                "ki_rate": 0.002,
                "kd_rate": 0.004,
                "i_limit": 0.5,
            }],
        ),
    ])
