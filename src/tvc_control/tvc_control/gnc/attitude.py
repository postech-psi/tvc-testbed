"""
Three-axis attitude cascade: attitude error -> body rate -> moment -> actuators.
================================================================================
Moved verbatim from physics.py.
"""

from .params import VehicleParams, ControlGains
from .mathx import attitude_error, euler_to_quat
from .pid import PID
from .allocation import Allocation, allocate


class AttitudeController:
    """
    Three-axis cascade:
        attitude error -> desired body rate  (outer, P)
        rate error     -> desired moment     (inner, PID)
        desired moment -> actuator commands  (allocation, §4b)

    All three axes are closed here. The LATERAL pair (body x, y) is driven by
    the gimbal; the ROLL channel (body z) is driven by tau_P, the differential
    prop reaction torque. See the axis-naming note in the module docstring:
    pitch_des/yaw_des are the lateral setpoints, roll_des is the one the
    controller-design note calls pitch.

    The roll channel used to be left at zero here on the grounds that the
    gimbal cannot produce M_z. That premise is right and the conclusion was
    wrong: the gimbal cannot, but the props can, and on this airframe they
    can do it hard -- Iz is 11.6x smaller than Ix, so 0.11 N*m of tau_P buys
    56 rad/s^2 about z against 15 rad/s^2 about x at full lateral authority.
    """

    def __init__(self, params: VehicleParams, gains: ControlGains):
        self.p = params
        self.gains = gains
        self.pid_pitch_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_yaw_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_roll_angle = PID(kp=gains.kp_angle_roll, ki=0.0, kd=0.0)
        self.pid_pitch_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_yaw_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_roll_rate = PID(kp=gains.kp_rate_roll, ki=gains.ki_rate_roll,
                                  kd=gains.kd_rate_roll, i_limit=gains.i_limit_roll)
        # Last allocation, exposed so callers that need the motor commands (or
        # the saturation flags, for an outer loop's own anti-windup) can read
        # them without changing update()'s return type.
        self.last_alloc = Allocation()

    def reset(self):
        for pid in (self.pid_pitch_angle, self.pid_yaw_angle, self.pid_roll_angle,
                    self.pid_pitch_rate, self.pid_yaw_rate, self.pid_roll_rate):
            pid.reset()
        self.last_alloc = Allocation()

    def desired_moment(self, q, omega, pitch_des, yaw_des, roll_des, dt):
        """Attitude + rate cascade -> desired body moment (M_x, M_y, M_z) [N*m].

        The attitude error is the quaternion form 2*sgn(qe_w)*qe_v, not an
        Euler difference. Both are the same to first order and this vehicle
        flies at a few degrees, so the change is small -- but the Euler path
        carried an arcsin singularity on one specific axis, which meant the
        axis NAMES carried a stability caveat. After the rename that axis is
        called "roll", and a reader would have to know which of three
        similar-looking channels was the fragile one. This form has no
        preferred axis, so the caveat disappears instead of moving.

        The setpoint is still specified as Euler angles because that is what a
        human types; it is converted here, once, at the boundary.
        """
        e = attitude_error(q, euler_to_quat(pitch_des, yaw_des, roll_des))
        sat = self.last_alloc

        pitch_rate_des = self.pid_pitch_angle.update(e[0], dt)
        yaw_rate_des = self.pid_yaw_angle.update(e[1], dt)
        roll_rate_des = self.pid_roll_angle.update(e[2], dt)

        # Freeze the lateral integrators when the gimbal is on its stops, and
        # the roll one when tau_P is capped -- see PID.update / Allocation.
        # The rate loop outputs ANGULAR ACCELERATION, not torque; inertia is
        # applied here and nowhere else. Two reasons this matters:
        #
        #   Gains become comparable. sim/hover.py computed tau = I*KP_RATE*err
        #   and physics.py computed tau = kp_rate*err, so the two "rate gains"
        #   differed by a factor of I and looked 25x apart when they were not
        #   describing the same quantity at all. In these units they can be put
        #   side by side, which is what makes adopting the flown set possible.
        #
        #   The roll gain stops rotting. kp_rate_roll was a hand-scaled copy
        #   of kp_rate carrying an Iz/Ix factor baked in, so re-measuring Iz
        #   would have silently changed the loop bandwidth. Now the scaling is
        #   explicit and follows the measurement.
        #
        # The DIAGONAL inertia is used for this mapping even though the plant
        # integrates the full tensor: this is gain scheduling, not dynamics, and
        # a controller that inverts its own model's cross terms is claiming a
        # model accuracy the mass budget does not support.
        alpha_x = self.pid_pitch_rate.update(pitch_rate_des - omega[0], dt,
                                            freeze=sat.gimbal_saturated)
        alpha_y = self.pid_yaw_rate.update(yaw_rate_des - omega[1], dt,
                                             freeze=sat.gimbal_saturated)
        alpha_z = self.pid_roll_rate.update(roll_rate_des - omega[2], dt,
                                             freeze=sat.roll_saturated)
        return (self.p.Ix * alpha_x, self.p.Iy * alpha_y, self.p.Iz * alpha_z)

    def update(self, q, omega, pitch_des, yaw_des, T_des, dt, roll_des=0.0):
        """Backward-compatible entry point: returns the gimbal command only.

        The full allocation -- tau_P, per-rotor thrusts, saturation flags --
        lands on self.last_alloc. Kept this shape because controller_node and
        the GUI call update() and use the return value directly as the servo
        command; adding a tuple return would have broken both.
        """
        M = self.desired_moment(q, omega, pitch_des, yaw_des, roll_des, dt)
        self.last_alloc = allocate(M, T_des, self.p)
        return self.last_alloc.delta_cmd
