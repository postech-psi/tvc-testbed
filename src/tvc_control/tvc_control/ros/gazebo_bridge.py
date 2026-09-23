"""Translate actuator commands to Gazebo joint angles and fictitious rotor speeds."""
import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from tvc_msgs.msg import ActuatorCommand
from tvc_control.config import load, load_vehicle_params
from tvc_control.control.mathx import clamp
import math


def rotor_speeds(thrust_n, tau_p_nm, motor_constant, moment_constant,
                 max_rot_velocity):
    """Convert thrust/roll moment to plugin speeds; these are not physical RPM."""
    split = tau_p_nm / moment_constant
    t_a = 0.5 * (thrust_n - split)
    t_b = 0.5 * (thrust_n + split)
    wa = math.sqrt(max(t_a, 0.0) / motor_constant)
    wb = math.sqrt(max(t_b, 0.0) / motor_constant)
    return (min(wa, max_rot_velocity), min(wb, max_rot_velocity))


class GazeboBridgeNode(Node):
    """Send one ActuatorCommand to Gazebo's three actuator topics."""

    def __init__(self):
        super().__init__('gazebo_bridge_node')

        self.params = load_vehicle_params()
        rotors = load().raw['rotors']
        self.motor_constant = rotors['motor_constant']
        self.moment_constant = rotors['moment_constant']
        self.max_rot_velocity = rotors['max_rot_velocity']
        self.lo = self.params.delta_min
        self.hi = self.params.delta_max

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.delta1_pub = self.create_publisher(
            Float64, '/tvc_vehicle/gimbal_inner_cmd', qos)
        self.delta2_pub = self.create_publisher(
            Float64, '/tvc_vehicle/gimbal_outer_cmd', qos)
        self.motor_pub = self.create_publisher(
            Actuators, '/tvc_vehicle/command/motor_speed', qos)

        self.create_subscription(ActuatorCommand, '/ctrl/actuator_cmd',
                                 self.on_cmd, qos)

        self.get_logger().info(
            'gazebo_bridge_node: gimbal travel inner %.2f..%.2f, outer %.2f..%.2f deg'
            % (self.lo[0] * 57.2957795, self.hi[0] * 57.2957795,
               self.lo[1] * 57.2957795, self.hi[1] * 57.2957795))

    def on_cmd(self, msg: ActuatorCommand):
        """Clamp per ring, then publish the two joint commands and the rotor speeds."""
        if msg.axis_convention != ActuatorCommand.AXIS_CONVENTION_ROCKET_V2:
            raise SystemExit(
                'axis convention mismatch: controller sent %d, this bridge '
                'speaks %d. See docs/GUIDE.md.'
                % (msg.axis_convention,
                   ActuatorCommand.AXIS_CONVENTION_ROCKET_V2))

        # Clamp per ring, per sign. The allocator already scales the pair to
        # preserve torque direction; this is the last line of defence against a
        # command that would otherwise be truncated by the joint stop, where the
        # truncation rotates the realized torque instead of shrinking it.
        d1 = clamp(msg.gimbal_inner_rad, self.lo[0], self.hi[0])
        d2 = clamp(msg.gimbal_outer_rad, self.lo[1], self.hi[1])
        self.delta1_pub.publish(Float64(data=float(d1)))
        self.delta2_pub.publish(Float64(data=float(d2)))

        act = Actuators()
        act.header.stamp = msg.header.stamp
        wa, wb = rotor_speeds(
            msg.thrust_n, msg.tau_p_nm, self.motor_constant,
            self.moment_constant, self.max_rot_velocity)
        act.velocity = [float(wa), float(wb)]
        self.motor_pub.publish(act)


def main(args=None):
    """ROS 2 entry point for `ros2 run tvc_control gazebo_bridge_node`."""
    rclpy.init(args=args)
    node = GazeboBridgeNode()
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
