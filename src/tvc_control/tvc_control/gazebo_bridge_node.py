"""
gazebo_bridge_node.py
=====================
Adapter between the controller's tvc_msgs/GimbalCommand and what Gazebo's
plugins accept. Gazebo does not speak tvc_msgs, and ros_gz_bridge only does
type-for-type translation, so something has to do the mapping.

Also holds the two coax rotors at a hover speed. The gimbal only redirects
thrust -- it cannot create any -- so with the rotors stopped the vehicle has
no control authority at all and every attitude test is meaningless.

Hover speed comes from the bench data, not a guess. Max thrust is taken from
the 2026-07-20 runs, the only ones reaching full throttle (PWM 2000); later
sessions stop at 1850 us. Those drove both rotors from one signal -- the
BALANCED case, which is the correct basis since unequal rotors make net yaw
torque, the one axis the gimbal cannot trim. Three runs gave 18.07 / 19.34 /
20.23 N; 20.0 N is adopted. Hover needs ~13.03 N at 1.328 kg, so
    omega_hover = 1100 * sqrt(13.03 / 20.0) = 888 rad/s
i.e. ~81% of max, T/W ~ 1.54 -- a workable margin. See docs/MASS_BUDGET.md.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from actuator_msgs.msg import Actuators
from std_msgs.msg import Float64
from tvc_msgs.msg import GimbalCommand

MAX_ROT_VELOCITY = 1100.0     # rad/s, must match the SDF
THRUST_AT_MAX_N = 20.0        # N, combined, balanced, full throttle (2026-07-20)
WEIGHT_N = 1.328 * 9.81


class GazeboBridgeNode(Node):
    def __init__(self):
        super().__init__("gazebo_bridge_node")

        self.declare_parameter("thrust_fraction", 1.0)
        self.declare_parameter("rotor_rate_hz", 100.0)

        frac = self.get_parameter("thrust_fraction").value
        target_thrust = WEIGHT_N * frac
        self.omega_hover = MAX_ROT_VELOCITY * np.sqrt(
            min(target_thrust / THRUST_AT_MAX_N, 1.0))

        if target_thrust > THRUST_AT_MAX_N:
            self.get_logger().warn(
                "requested %.2f N exceeds measured max thrust %.2f N -- "
                "commanding max and the vehicle will sink"
                % (target_thrust, THRUST_AT_MAX_N))

        self.get_logger().info(
            "hover omega = %.1f rad/s (%.0f%% of max) for %.2f N"
            % (self.omega_hover, 100 * self.omega_hover / MAX_ROT_VELOCITY,
               target_thrust))

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pitch_pub = self.create_publisher(
            Float64, "/tvc_vehicle/gimbal_pitch", qos)
        self.roll_pub = self.create_publisher(
            Float64, "/tvc_vehicle/gimbal_roll", qos)
        self.motor_pub = self.create_publisher(
            Actuators, "/tvc_vehicle/command/motor_speed", qos)

        # Same topic controller_node already publishes on, so the controller
        # is untouched by the Gazebo swap.
        self.create_subscription(
            GimbalCommand, "/ctrl/gimbal_cmd", self.on_gimbal, qos)

        rate = self.get_parameter("rotor_rate_hz").value
        self.create_timer(1.0 / rate, self.publish_rotors)

    def on_gimbal(self, msg):
        # delta1 = pitch-plane, delta2 = roll-plane (physics.py convention)
        self.pitch_pub.publish(Float64(data=float(msg.delta1)))
        self.roll_pub.publish(Float64(data=float(msg.delta2)))

    def publish_rotors(self):
        m = Actuators()
        m.velocity = [self.omega_hover, self.omega_hover]
        self.motor_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
