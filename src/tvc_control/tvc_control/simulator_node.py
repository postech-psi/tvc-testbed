"""
simulator_node -- the analytic plant, wrapped in ROS 2.
================================================================================
The non-Gazebo plant. It exists so the ROS2 control path can be exercised
without a physics engine, and -- more usefully -- so that `phase4.launch.py` and
`gazebo.launch.py` differ ONLY in which plant process is running. Same
controller, same message, same state topic; swap the plant and compare. That is
the whole point of the split, made operational.

Two things it used to get wrong:

  IT DROPPED HALF THE COMMAND. dynamics() was called without tau_p and with
  thrust pinned at m*g, so the roll channel did not exist on this path and there
  was no altitude loop -- regardless of what the controller had computed.

  IT PUBLISHED THE WRONG SENSOR. It published an IMU, which carries no position
  or velocity, so no outer loop could ever close. It publishes Odometry now, on
  the same topic name Gazebo's OdometryPublisher uses, which is what makes the
  two plants interchangeable.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.integrate import solve_ivp

from tvc_msgs.msg import ActuatorCommand
from tvc_control.config import load_vehicle_params, load
from tvc_control.gnc.mathx import euler_to_quat, quat_normalize
from tvc_control.plant.actuators import ActuatorChain
from tvc_control.plant.rigidbody import dynamics

RETIRED_PARAMS = ('gimbal_rate_max_deg',)


class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator_node')

        for name in RETIRED_PARAMS:
            self.declare_parameter(name, rclpy.Parameter.Type.NOT_SET)
            if self.get_parameter(name).type_ != rclpy.Parameter.Type.NOT_SET:
                raise SystemExit(
                    "parameter '%s' was retired: the gimbal has two rings with "
                    "different measured slew rates (403 and 235 deg/s). Edit "
                    "gimbal.axes.*.rate_max_deg in vehicle_params.yaml. "
                    "See docs/CONVENTIONS.md." % name)

        self.declare_parameter('dt', 0.004)
        self.declare_parameter('init_roll_deg', 3.0)
        self.declare_parameter('init_pitch_deg', -4.0)
        self.declare_parameter('init_z', 2.0)

        self.dt = float(self.get_parameter('dt').value)
        self.params = load_vehicle_params()
        md = (load().raw.get('motor_dynamics') or {})
        self.chain = ActuatorChain(
            self.params,
            motor_model=md.get('model', 'first_order'),
            motor_tau_s=md.get('tau_s', 0.10),
            motor_deadtime_s=md.get('deadtime_s', 0.0))

        q0 = euler_to_quat(
            np.deg2rad(self.get_parameter('init_roll_deg').value),
            np.deg2rad(self.get_parameter('init_pitch_deg').value), 0.0)
        z0 = float(self.get_parameter('init_z').value)
        self.x = np.concatenate([[0, 0, z0], [0, 0, 0], q0, [0, 0, 0]])
        self.t = 0.0

        # Hold hover thrust until the first command arrives. Zero would make the
        # vehicle free-fall for however long the controller takes to start,
        # which then reads as a control failure.
        self.cmd = None
        self.T_hover = self.params.m * self.params.g

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(ActuatorCommand, '/ctrl/actuator_cmd',
                                 self.on_cmd, qos)
        self.odom_pub = self.create_publisher(
            Odometry, '/model/tvc_vehicle/odometry', qos)
        self.create_timer(self.dt, self.tick)

        self.get_logger().info('simulator_node: dt=%.4f s, z0=%.2f m' % (self.dt, z0))

    def on_cmd(self, msg: ActuatorCommand):
        if msg.axis_convention != ActuatorCommand.AXIS_CONVENTION_LEGACY_QUADCOPTER:
            raise SystemExit(
                'axis convention mismatch: controller sent %d, this plant speaks '
                '%d. See docs/CONVENTIONS.md.'
                % (msg.axis_convention,
                   ActuatorCommand.AXIS_CONVENTION_LEGACY_QUADCOPTER))
        self.cmd = msg

    def tick(self):
        if self.cmd is None:
            delta_cmd, T_cmd, tau_p_cmd = np.zeros(2), self.T_hover, 0.0
        else:
            delta_cmd = np.array([self.cmd.gimbal_delta1_rad,
                                  self.cmd.gimbal_delta2_rad])
            T_cmd, tau_p_cmd = self.cmd.thrust_n, self.cmd.tau_p_nm

        # The same actuator chain the analytic harness uses -- gimbal slew and
        # deadtime, motor lag -- so the two cannot model different subsets.
        delta, T, tau_p = self.chain.update(delta_cmd, T_cmd, tau_p_cmd, self.dt)

        sol = solve_ivp(dynamics, [self.t, self.t + self.dt], self.x,
                        args=(T, delta, self.params, tau_p),
                        method='RK45', max_step=self.dt / 4)
        self.x = sol.y[:, -1]
        self.x[6:10] = quat_normalize(self.x[6:10])
        self.t += self.dt
        self.publish_odometry()

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        p = msg.pose.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in self.x[0:3])
        # gnc uses (qw, qx, qy, qz); geometry_msgs/Quaternion is (x, y, z, w).
        qw, qx, qy, qz = self.x[6:10]
        p.orientation.w, p.orientation.x = float(qw), float(qx)
        p.orientation.y, p.orientation.z = float(qy), float(qz)
        t = msg.twist.twist
        t.linear.x, t.linear.y, t.linear.z = (float(v) for v in self.x[3:6])
        t.angular.x, t.angular.y, t.angular.z = (float(v) for v in self.x[10:13])
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
