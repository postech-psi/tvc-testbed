"""
Altitude cascade with tilt feedforward.
================================================================================
Moved verbatim from physics.py.
"""

from .params import VehicleParams, ControlGains
from .mathx import quat_to_rotmat, thrust_axis, clamp, isclose
from .pid import PID


class AltitudeController:
    """Cascade P (altitude -> climb rate) + PID (climb rate -> accel) -> thrust.

    The tilt compensation is the part that matters. Vertical dynamics are

        m*z_ddot = T * (R(q) n_hat) . z_inertial - m*g

    so the useful fraction of thrust is cos(theta), theta being the angle
    between the thrust axis and vertical. Dividing the thrust command by that
    same projection is FEEDFORWARD: it cancels the altitude sag from a tilt at
    the instant the tilt happens, instead of waiting for the altitude error to
    grow large enough for feedback to notice. Without it, every attitude
    maneuver shows up as an altitude dip.

    The divisor is floored (cos_min) so a large or briefly bad attitude
    estimate cannot demand unbounded thrust; at the floor the vehicle simply
    accepts the sag, which is the safe failure.
    """

    def __init__(self, params: VehicleParams, gains: ControlGains, cos_min=0.7,
                 tilt_compensation=True):
        self.p = params
        self.gains = gains
        self.cos_min = cos_min
        # tilt_compensation=False divides by 1 instead of the projection. Only
        # useful for the A/B in sim/validate_control.py that shows what the
        # feedforward is actually buying; flying without it is strictly worse.
        self.tilt_compensation = tilt_compensation
        self.pid_vz = PID(kp=gains.kp_vz, ki=gains.ki_vz, kd=gains.kd_vz,
                          i_limit=gains.i_limit_vz)
        self.thrust_saturated = False

    def reset(self):
        self.pid_vz.reset()
        self.thrust_saturated = False

    def update(self, z, vz, q, delta, z_des, dt):
        """Altitude state + current gimbal -> total thrust command [N]."""
        vz_des = clamp(self.gains.kp_alt * (z_des - z),
                       -self.gains.vz_max, self.gains.vz_max)
        # Anti-windup: hold the integrator whenever the previous command was
        # clipped by the feasible set (§4b priority: thrust first, but "first"
        # still ends at T_max).
        az_des = self.pid_vz.update(vz_des - vz, dt, freeze=self.thrust_saturated)

        # Projection of the (gimballed, then rotated) thrust axis onto vertical.
        if self.tilt_compensation:
            # (R @ n_hat) . z_inertial is just the third row of R dotted
            # with n_hat -- no matrix product needed, and writing it out
            # keeps the flight path free of a linear-algebra dependency.
            r2 = quat_to_rotmat(q)[2]
            n = thrust_axis(delta)
            proj = r2[0] * n[0] + r2[1] * n[1] + r2[2] * n[2]
            proj = max(proj, self.cos_min)
        else:
            proj = 1.0

        T_cmd = self.p.m * (self.p.g + az_des) / proj
        T_clipped = clamp(T_cmd, self.p.T_min, self.p.T_max)
        self.thrust_saturated = not isclose(T_clipped, T_cmd)
        return T_clipped
