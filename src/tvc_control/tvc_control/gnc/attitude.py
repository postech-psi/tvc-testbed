"""
Three-axis attitude cascade: attitude error -> body rate -> moment -> actuators.
================================================================================
Moved verbatim from physics.py.
"""

from .params import VehicleParams, ControlGains
from .mathx import quat_to_euler
from .pid import PID
from .allocation import Allocation, allocate


class AttitudeController:
    """
    Three-axis cascade:
        attitude error -> desired body rate  (outer, P)
        rate error     -> desired moment     (inner, PID)
        desired moment -> actuator commands  (allocation, §4b)

    All three axes are closed here. The LATERAL pair (body x, y) is driven by
    the gimbal; the AXIAL channel (body z) is driven by tau_P, the differential
    prop reaction torque. See the axis-naming note in the module docstring:
    roll_des/pitch_des are the lateral setpoints, axial_des is the one the
    controller-design note calls roll.

    The axial channel used to be left at zero here on the grounds that the
    gimbal cannot produce M_z. That premise is right and the conclusion was
    wrong: the gimbal cannot, but the props can, and on this airframe they
    can do it hard -- Iz is 11.6x smaller than Ix, so 0.11 N*m of tau_P buys
    56 rad/s^2 about z against 15 rad/s^2 about x at full lateral authority.
    """

    def __init__(self, params: VehicleParams, gains: ControlGains):
        self.p = params
        self.gains = gains
        self.pid_roll_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_pitch_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_axial_angle = PID(kp=gains.kp_angle_axial, ki=0.0, kd=0.0)
        self.pid_roll_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_pitch_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_axial_rate = PID(kp=gains.kp_rate_axial, ki=gains.ki_rate_axial,
                                  kd=gains.kd_rate_axial, i_limit=gains.i_limit_axial)
        # Last allocation, exposed so callers that need the motor commands (or
        # the saturation flags, for an outer loop's own anti-windup) can read
        # them without changing update()'s return type.
        self.last_alloc = Allocation()

    def reset(self):
        for pid in (self.pid_roll_angle, self.pid_pitch_angle, self.pid_axial_angle,
                    self.pid_roll_rate, self.pid_pitch_rate, self.pid_axial_rate):
            pid.reset()
        self.last_alloc = Allocation()

    def desired_moment(self, q, omega, roll_des, pitch_des, axial_des, dt):
        """Attitude + rate cascade -> desired body moment (M_x, M_y, M_z) [N*m].

        Euler angles are used for the OUTER loop error only. That is safe here
        and is not the gimbal-lock trap the module docstring warns about: the
        trap is propagating kinematics through Euler angles (this module never
        does -- see quat_kinematics), whereas an error measured at the few
        degrees this vehicle actually flies is nowhere near the +/-90 deg
        singularity. If the envelope ever grows, swap in the quaternion error
        2*sgn(qe_w)*qe_v; it agrees with this to first order.
        """
        roll, pitch, axial = quat_to_euler(q)
        sat = self.last_alloc

        roll_rate_des = self.pid_roll_angle.update(roll_des - roll, dt)
        pitch_rate_des = self.pid_pitch_angle.update(pitch_des - pitch, dt)
        axial_rate_des = self.pid_axial_angle.update(axial_des - axial, dt)

        # Freeze the lateral integrators when the gimbal is on its stops, and
        # the axial one when tau_P is capped -- see PID.update / Allocation.
        tau_x = self.pid_roll_rate.update(roll_rate_des - omega[0], dt,
                                          freeze=sat.gimbal_saturated)
        tau_y = self.pid_pitch_rate.update(pitch_rate_des - omega[1], dt,
                                           freeze=sat.gimbal_saturated)
        tau_z = self.pid_axial_rate.update(axial_rate_des - omega[2], dt,
                                           freeze=sat.axial_saturated)
        return (tau_x, tau_y, tau_z)

    def update(self, q, omega, roll_des, pitch_des, T_des, dt, axial_des=0.0):
        """Backward-compatible entry point: returns the gimbal command only.

        The full allocation -- tau_P, per-rotor thrusts, saturation flags --
        lands on self.last_alloc. Kept this shape because controller_node and
        the GUI call update() and use the return value directly as the servo
        command; adding a tuple return would have broken both.
        """
        M = self.desired_moment(q, omega, roll_des, pitch_des, axial_des, dt)
        self.last_alloc = allocate(M, T_des, self.p)
        return self.last_alloc.delta_cmd
