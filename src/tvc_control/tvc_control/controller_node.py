"""
controller_node.py
===================
Phase 4 controller node. Owns the cascaded PID (AttitudeController) from
tvc_control.physics -- nothing else. Subscribes to vehicle attitude,
computes the gimbal deflection command, publishes it.

PID gains and setpoints are ROS2 parameters (declare_parameter), not
hardcoded -- the same "explore the design space without editing code"
capability the Phase 2 GUI already gave, just reachable from a launch
file or `ros2 param set` instead of a form.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from tvc_msgs.msg import GimbalCommand

from tvc_control.physics import (ControlGains, AttitudeController,
                                 load_vehicle_params)


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('dt', 0.01)
        self.declare_parameter('roll_des_deg', 0.0)
        self.declare_parameter('pitch_des_deg', 5.0)
        self.declare_parameter('kp_angle', 4.0)
        self.declare_parameter('kp_rate', 0.02)
        self.declare_parameter('ki_rate', 0.002)
        self.declare_parameter('kd_rate', 0.004)
        self.declare_parameter('i_limit', 0.5)

        self.dt = self.get_parameter('dt').value
        self.roll_des = np.deg2rad(self.get_parameter('roll_des_deg').value)
        self.pitch_des = np.deg2rad(self.get_parameter('pitch_des_deg').value)

        gains = ControlGains(
            kp_angle=self.get_parameter('kp_angle').value,
            kp_rate=self.get_parameter('kp_rate').value,
            ki_rate=self.get_parameter('ki_rate').value,
            kd_rate=self.get_parameter('kd_rate').value,
            i_limit=self.get_parameter('i_limit').value,
        )
        self.params = load_vehicle_params()
        self.controller = AttitudeController(self.params, gains)
        self.T_hover = self.params.m * self.params.g

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.attitude_sub = self.create_subscription(
            Imu, '/sim/vehicle_attitude', self.on_attitude, qos)
        self.gimbal_pub = self.create_publisher(
            GimbalCommand, '/ctrl/gimbal_cmd', qos)

        self.get_logger().info(
            f'controller_node started: roll_des={np.rad2deg(self.roll_des):.1f}deg, '
            f'pitch_des={np.rad2deg(self.pitch_des):.1f}deg')

    def on_attitude(self, msg: Imu):
        # geometry_msgs/Quaternion is (x, y, z, w); physics.py uses (qw, qx, qy, qz).
        q = np.array([
            msg.orientation.w, msg.orientation.x,
            msg.orientation.y, msg.orientation.z,
        ])
        omega = np.array([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        ])

        delta = self.controller.update(
            q, omega, self.roll_des, self.pitch_des, self.T_hover, self.dt)

        cmd = GimbalCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body'
        cmd.delta1 = float(delta[0])
        cmd.delta2 = float(delta[1])
        self.gimbal_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
