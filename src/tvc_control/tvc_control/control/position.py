"""Horizontal position and velocity errors become pitch and yaw targets."""


from .mathx import clamp


class PositionController:
    """Horizontal position + velocity error -> commanded tilt, in radians.

    Returns (pitch_des, yaw_des) in radians -- pitch about body x, yaw about
    body y, per docs/GUIDE.md. Both are clamped to gains.max_tilt.
    """

    def __init__(self, gains):
        self.gains = gains

    def update(self, pos_i, vel_i, pos_des):
        """Horizontal position and velocity error -> (pitch_des, yaw_des) in rad.

        pos_i, vel_i : inertial position [m] and velocity [m/s], (x, y, z)
        pos_des      : inertial target, (x, y, z); z is ignored -- the altitude
                       loop owns it
        """
        ex = pos_i[0] - pos_des[0]
        ey = pos_i[1] - pos_des[1]
        g = self.gains
        lim = g.max_tilt

        # Sign derivation, kept verbatim because both signs were wrong once and
        # this comment is what caught it:
        #
        #   Rotating body +z into the world: +yaw tips thrust toward +x, and
        #   +pitch tips it toward -y. So to come BACK to the origin the demands
        #   carry opposite signs to each other:
        #       x > 0  needs -x force -> negative yaw
        #       y > 0  needs -y force -> positive pitch
        #   Getting either backwards turns this loop into positive feedback and
        #   the vehicle accelerates away instead of returning.
        yaw_des = clamp(-(g.kp_pos * ex + g.kd_pos * vel_i[0]), -lim, lim)
        pitch_des = clamp(+(g.kp_pos * ey + g.kd_pos * vel_i[1]), -lim, lim)
        return pitch_des, yaw_des
