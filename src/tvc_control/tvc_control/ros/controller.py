"""ROS odometry in, shared controller calculations, actuator commands and CSV log out."""
import csv
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tvc_msgs.msg import ActuatorCommand
from tvc_control.config import load_gains, load_vehicle_params
from tvc_control.control.controller import TvcController
from tvc_control.control.mathx import quat_rotate, quat_to_euler
from tvc_control.control.types import ControlMode, EstimatedState, Setpoint

class ControllerNode(Node):
    """Subscribes to odometry, runs TvcController on a timer, publishes one
    ActuatorCommand. Owns no control math.
    """

    def __init__(self):
        super().__init__('controller_node')

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
        self.declare_parameter('log_path', '')

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
        self._log_start = None
        self._log_file = None
        log_path = self.get_parameter('log_path').value
        if log_path:
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(log_path, 'w', newline='', encoding='utf-8')
            self._log_file.write('# axis_convention: rocket_v2\n')
            self._log = csv.writer(self._log_file)
            self._log.writerow(['t_s', 'z_m', 'x_m', 'y_m', 'pitch_deg', 'yaw_deg',
                                'roll_deg', 'gimbal_outer_deg', 'gimbal_inner_deg',
                                'thrust_N', 'tau_p_Nm'])
            self._last_flush = 0.0
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
        # ROS stores quaternion fields x/y/z/w; control uses w/x/y/z.
        # Gazebo odometry twist is in the body frame; convert linear velocity.
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
        if self._log_file is not None:
            stamp = self._state.stamp_s
            if self._log_start is None:
                self._log_start = stamp
            t = stamp - self._log_start
            x, y, z = self._state.pos_i
            pitch, yaw, roll = np.rad2deg(quat_to_euler(self._state.quat))
            self._log.writerow([t, z, x, y, pitch, yaw, roll,
                                np.rad2deg(cmd.gimbal_outer_rad),
                                np.rad2deg(cmd.gimbal_inner_rad), cmd.thrust_n, cmd.tau_p_nm])
            if t - self._last_flush >= 1.0:
                self._log_file.flush()
                self._last_flush = t

    def destroy_node(self):
        if self._log_file is not None:
            self._log_file.close()
        return super().destroy_node()


def main(args=None):
    """ROS 2 entry point for `ros2 run tvc_control controller_node`."""
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
