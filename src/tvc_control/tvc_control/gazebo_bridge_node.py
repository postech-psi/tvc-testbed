"""
gazebo_bridge_node -- the Gazebo HAL, as a ROS 2 node. No control lives here.
================================================================================
Fans one ActuatorCommand out to the three topics gz-sim's plugins listen on:
two joint position controllers and the multicopter motor model. The conversion
itself is tvc_control.hal.gazebo, shared with sim/hover.py, so the ROS path and
the standalone gz-transport path cannot drift apart.

What this node used to do, and why it mattered: it held BOTH rotors at one fixed
hover speed computed from a thrust_fraction parameter, ignoring the controller
entirely. Equal rotor speeds means zero differential prop torque, which on this
vehicle means zero authority about the thrust axis -- the ROS2 path had no roll
control at all, by construction. It also passed the gimbal angles through with
no clamp, so a command outside the joint's +/-0.1222 rad stops was silently
truncated by Gazebo instead of by the allocator that knows the travel is
asymmetric.

The rotor speeds it publishes are a CONTROL ALLOCATION VARIABLE, not a physical
RPM -- see hal/gazebo.py. The old docstring here argued for a 20.0 N maximum
thrust and an 888 rad/s hover speed; both were superseded (17.79 N measured, and
the speed now depends on solver constants), which is exactly the kind of stale
duplicate the single-source-of-truth work removed.
"""
import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from tvc_msgs.msg import ActuatorCommand
from tvc_control.config import load_vehicle_params
from tvc_control.gnc.mathx import clamp


class GazeboBridgeNode(Node):

    def __init__(self):
        super().__init__('gazebo_bridge_node')

        self.params = load_vehicle_params()
        self.lo = self.params.delta_min
        self.hi = self.params.delta_max

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.delta1_pub = self.create_publisher(
            Float64, '/tvc_vehicle/gimbal_pitch', qos)
        self.delta2_pub = self.create_publisher(
            Float64, '/tvc_vehicle/gimbal_roll', qos)
        self.motor_pub = self.create_publisher(
            Actuators, '/tvc_vehicle/command/motor_speed', qos)

        self.create_subscription(ActuatorCommand, '/ctrl/actuator_cmd',
                                 self.on_cmd, qos)

        self.get_logger().info(
            'gazebo_bridge_node: gimbal travel inner %.2f..%.2f, outer %.2f..%.2f deg'
            % (self.lo[0] * 57.2957795, self.hi[0] * 57.2957795,
               self.lo[1] * 57.2957795, self.hi[1] * 57.2957795))

    def on_cmd(self, msg: ActuatorCommand):
        if msg.axis_convention != ActuatorCommand.AXIS_CONVENTION_LEGACY_QUADCOPTER:
            raise SystemExit(
                'axis convention mismatch: controller sent %d, this bridge '
                'speaks %d. See docs/CONVENTIONS.md.'
                % (msg.axis_convention,
                   ActuatorCommand.AXIS_CONVENTION_LEGACY_QUADCOPTER))

        # Clamp per ring, per sign. The allocator already scales the pair to
        # preserve torque direction; this is the last line of defence against a
        # command that would otherwise be truncated by the joint stop, where the
        # truncation rotates the realized torque instead of shrinking it.
        d1 = clamp(msg.gimbal_delta1_rad, self.lo[0], self.hi[0])
        d2 = clamp(msg.gimbal_delta2_rad, self.lo[1], self.hi[1])
        self.delta1_pub.publish(Float64(data=float(d1)))
        self.delta2_pub.publish(Float64(data=float(d2)))

        act = Actuators()
        act.header.stamp = msg.header.stamp
        act.velocity = [float(msg.rotor_a_speed_rads),
                        float(msg.rotor_b_speed_rads)]
        self.motor_pub.publish(act)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
