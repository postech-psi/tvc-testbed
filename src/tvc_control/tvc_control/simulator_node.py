"""
simulator_node.py
==================
Phase 4 "vehicle" node. Owns the 6-DOF rigid-body state and the gimbal
actuator model from tvc_control.physics -- nothing else. Subscribes to
commanded gimbal deflections, integrates one physics step per timer tick,
and publishes the resulting attitude.

This node is the swappable seam described in the project roadmap: Phase 5
replaces this whole node with Gazebo + PX4 SITL, and neither the topic
names nor controller_node have to change for that swap to work.
"""

import numpy as np
from scipy.integrate import solve_ivp

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from tvc_msgs.msg import GimbalCommand

from tvc_control.physics import (
    load_vehicle_params,
    GimbalActuator,
    dynamics,
    quat_normalize,
    euler_to_quat,
)


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')

        self.declare_parameter('dt', 0.01)
        self.declare_parameter('init_roll_deg', 3.0)
        self.declare_parameter('init_pitch_deg', -4.0)
        self.declare_parameter('gimbal_rate_max_deg', rclpy.Parameter.Type.DOUBLE)

        self.dt = self.get_parameter('dt').value
        init_roll_deg = self.get_parameter('init_roll_deg').value
        init_pitch_deg = self.get_parameter('init_pitch_deg').value
        gimbal_rate_max_deg = self.get_parameter_or(
            'gimbal_rate_max_deg', None)
        gimbal_rate_max_deg = (gimbal_rate_max_deg.value
                               if gimbal_rate_max_deg is not None else None)

        # gimbal_rate_max_deg is RETIRED. It was a scalar summary of a
        # per-ring fact, and since GimbalActuator started reading the measured
        # per-ring rates it has been silently ignored -- the parameter looked
        # live while doing nothing. Fail loudly instead of pretending.
        if gimbal_rate_max_deg is not None:
            raise SystemExit(
                "parameter 'gimbal_rate_max_deg' was retired: the gimbal has "
                "two rings with different measured slew rates (403 and 235 "
                "deg/s). Edit gimbal.axes.*.rate_max_deg in "
                "tvc_control/vehicle_params.yaml. See docs/CONVENTIONS.md.")
        self.params = load_vehicle_params()
        self.gimbal = GimbalActuator(self.params)
        self.T_hover = self.params.m * self.params.g

        q0 = euler_to_quat(np.deg2rad(init_roll_deg), np.deg2rad(init_pitch_deg), 0.0)
        self.x = np.concatenate([[0, 0, 0], [0, 0, 0], q0, [0, 0, 0]])
        self.t = 0.0

        self.delta_cmd = np.zeros(2)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.gimbal_sub = self.create_subscription(
            GimbalCommand, '/ctrl/gimbal_cmd', self.on_gimbal_cmd, qos)
        self.attitude_pub = self.create_publisher(
            Imu, '/sim/vehicle_attitude', qos)

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'simulator_node started: init_roll={init_roll_deg}deg, '
            f'init_pitch={init_pitch_deg}deg, dt={self.dt}s')

    def on_gimbal_cmd(self, msg: GimbalCommand):
        self.delta_cmd = np.array([msg.delta1, msg.delta2])

    def tick(self):
        delta = self.gimbal.update(self.delta_cmd, self.dt)

        sol = solve_ivp(
            dynamics, [self.t, self.t + self.dt], self.x,
            args=(self.T_hover, delta, self.params),
            method='RK45', max_step=self.dt / 4)
        self.x = sol.y[:, -1]
        self.x[6:10] = quat_normalize(self.x[6:10])
        self.t += self.dt

        self.publish_attitude()

    def publish_attitude(self):
        q = self.x[6:10]
        omega = self.x[10:13]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'
        # quat_normalize / euler_to_quat use (qw, qx, qy, qz) ordering;
        # geometry_msgs/Quaternion is (x, y, z, w).
        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]
        msg.angular_velocity.x = omega[0]
        msg.angular_velocity.y = omega[1]
        msg.angular_velocity.z = omega[2]
        self.attitude_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
