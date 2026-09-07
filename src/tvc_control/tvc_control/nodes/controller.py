"""
controller_node -- the flight code, wrapped in ROS 2 and nothing else.
================================================================================
Owns no control math. It subscribes to state, calls TvcController, and publishes
what came back. Every loop -- position, altitude, attitude, allocation -- lives
in tvc_control.gnc and is the same code the analytic harness and the standalone
Gazebo harness run, which is what makes a result obtained in one of them evidence
about the others.

Three things this node used to get wrong, all fixed here:

  IT THREW AWAY MOST OF ITS OWN OUTPUT. It called AttitudeController.update(),
  published the two gimbal angles, and discarded last_alloc -- the thrust
  command, tau_P, the per-rotor split and the saturation flags. There was no
  motor topic at all, so the ROS2 path had no altitude loop and, since the
  bridge held both rotors at one speed, exactly zero roll authority
  (roll being rotation about the thrust axis -- see docs/4-CONVENTIONS.md).

  IT LISTENED TO THE WRONG SENSOR. It subscribed to the IMU, which carries no
  position and no velocity, so altitude and position control were not merely
  unimplemented but impossible. Odometry is the single state source now, shared
  by the direct Gazebo harness and both ROS plant paths.

  IT LIED ABOUT dt. It ran off IMU callbacks at 250 Hz while passing a hardcoded
  dt = 0.01 into the PIDs, a 2.5x error in every integral and derivative term.
  Control now runs on a timer, dt is measured from the clock, and the callback
  only caches state.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tvc_msgs.msg import ActuatorCommand
from tvc_control.config import load_gains, load_vehicle_params
from tvc_control.gnc.controller import TvcController
from tvc_control.gnc.mathx import quat_rotate
from tvc_control.gnc.types import ControlMode, EstimatedState, Setpoint
from tvc_control.nodes import reject_retired_parameters

# Retired parameter names. Each one either changed meaning or stopped doing
# anything, and a parameter that looks live while being ignored is worse than
# one that is gone -- it makes a launch file document a control decision that
# is not happening. See docs/4-CONVENTIONS.md.
_AXIS = ("the axis names changed with the rocket convention: roll is now the "
         "THRUST axis. Use att_pitch_des_deg / att_yaw_des_deg / "
         "att_roll_des_deg, which mean body x / body y / body z.")
RETIRED_PARAMS = {
    'gimbal_rate_max_deg': "the two rings have different measured slew rates "
                           "(403 and 235 deg/s). Set gimbal.axes.*.rate_max_deg "
                           "in vehicle_params.yaml.",
    'roll_des_deg': _AXIS, 'pitch_des_deg': _AXIS, 'axial_des_deg': _AXIS,
    'init_roll_deg': _AXIS, 'init_pitch_deg': _AXIS, 'init_axial_deg': _AXIS,
}


class ControllerNode(Node):
    """Subscribes to odometry, runs TvcController on a timer, publishes one
    ActuatorCommand. Owns no control math.
    """

    def __init__(self):
        super().__init__('controller_node')

        reject_retired_parameters(self, RETIRED_PARAMS)

        self.declare_parameter('rate_hz', 250.0)
        self.declare_parameter('gain_profile', '')
        self.declare_parameter('att_pitch_des_deg', 0.0)
        self.declare_parameter('att_yaw_des_deg', 0.0)
        self.declare_parameter('att_roll_des_deg', 0.0)
        self.declare_parameter('altitude_hold', False)
        self.declare_parameter('position_hold', False)
        self.declare_parameter('z_des', 2.0)
        self.declare_parameter('x_des', 0.0)
        self.declare_parameter('y_des', 0.0)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        profile = self.get_parameter('gain_profile').value or None

        self.params = load_vehicle_params()
        gains = load_gains(profile)
        self.controller = TvcController(self.params, gains, ControlMode(
            altitude_hold=bool(self.get_parameter('altitude_hold').value),
            position_hold=bool(self.get_parameter('position_hold').value),
        ))
        self.setpoint = Setpoint(
            pitch_des=np.deg2rad(self.get_parameter('att_pitch_des_deg').value),
            yaw_des=np.deg2rad(self.get_parameter('att_yaw_des_deg').value),
            roll_des=np.deg2rad(self.get_parameter('att_roll_des_deg').value),
            z_des=float(self.get_parameter('z_des').value),
            pos_des=(float(self.get_parameter('x_des').value),
                     float(self.get_parameter('y_des').value), 0.0),
        )

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self._state = None
        self._last_t = None
        self.create_subscription(Odometry, '/model/tvc_vehicle/odometry',
                                 self.on_odometry, qos)
        self.cmd_pub = self.create_publisher(ActuatorCommand, '/ctrl/actuator_cmd', qos)
        self.create_timer(1.0 / self.rate_hz, self.control_step)

        self.get_logger().info(
            'controller_node at %.0f Hz, gains=%s, altitude_hold=%s, position_hold=%s'
            % (self.rate_hz, profile or '<file default>',
               self.controller.mode.altitude_hold,
               self.controller.mode.position_hold))

    def on_odometry(self, msg: Odometry):
        """Cache only. Control runs on the timer -- see the dt note above."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # geometry_msgs/Quaternion is (x, y, z, w); gnc uses (qw, qx, qy, qz).
        #
        # twist is in child_frame_id -- the BODY frame, per REP-105 -- and both
        # publishers on this topic obey that: gz-sim's OdometryPublisher because
        # it differences the pose in the body frame, simulator_node because it
        # is written to match. vel_i is inertial by definition, so rotate.
        quat = (q.w, q.x, q.y, q.z)
        self._state = EstimatedState(
            pos_i=(p.x, p.y, p.z), vel_i=quat_rotate(quat, (v.x, v.y, v.z)),
            quat=quat, omega_b=(w.x, w.y, w.z),
            stamp_s=stamp)

    def control_step(self):
        """The timer callback: one control step, at the configured rate."""
        if self._state is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        nominal = 1.0 / self.rate_hz
        if self._last_t is None:
            dt = nominal
        else:
            # Clamped to a sane band around nominal. A stalled executor or a
            # paused simulator can hand back a dt of seconds, and an unclamped
            # derivative term on that is a single enormous actuator command.
            dt = min(max(now - self._last_t, 0.2 * nominal), 5.0 * nominal)
        self._last_t = now

        cmd = self.controller.update(self._state, self.setpoint, dt)

        msg = ActuatorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'
        msg.axis_convention = ActuatorCommand.AXIS_CONVENTION_ROCKET_V2
        msg.gimbal_inner_rad = float(cmd.gimbal_inner_rad)
        msg.gimbal_outer_rad = float(cmd.gimbal_outer_rad)
        msg.motor_a = float(cmd.motor_a)
        msg.motor_b = float(cmd.motor_b)
        msg.thrust_n = float(cmd.thrust_n)
        msg.tau_p_nm = float(cmd.tau_p_nm)
        msg.sat_gimbal = bool(cmd.sat_gimbal)
        msg.sat_roll = bool(cmd.sat_roll)
        msg.sat_thrust = bool(cmd.sat_thrust)
        self.cmd_pub.publish(msg)


def main(args=None):
    """ROS 2 entry point for `ros2 run tvc_control controller_node`."""
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
