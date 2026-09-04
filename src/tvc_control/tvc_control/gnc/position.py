"""
Position hold: horizontal position error -> attitude setpoint.
================================================================================
Ported from sim/hover.py, which is the only controller in this project that has
ever flown. It outputs an ATTITUDE setpoint rather than a torque, so it stacks
outboard of the attitude cascade without touching it.

WHY THIS LOOP MUST BE MUCH SLOWER THAN THE ATTITUDE LOOP
    What diverges on this vehicle is position, not attitude. With the gimbal
    centred, thrust acts along body +z at r = (0,0,-L) from the CM, so tau =
    r x F = 0 regardless of orientation, and gravity acts at the CM and makes no
    torque at all: attitude is a double integrator, neutrally stable, not an
    inverted pendulum. (Believing thrust-below-CG either self-stabilises or
    diverges like a pendulum is the pendulum rocket fallacy in both directions;
    a pendulum has a ground pivot for gravity to act about and a flying vehicle
    does not.)

    Any tilt, however, puts a horizontal component on the thrust vector, which
    accelerates the vehicle sideways, which needs an opposite tilt to arrest.
    That is the loop that runs away. Letting it approach the attitude loop in
    bandwidth is what makes these vehicles wobble: at kp=0.22/kd=0.35 the two
    were only 6.5x apart and the vehicle settled into a +/-9 deg coning limit
    cycle at the attitude natural frequency, pitch and yaw 90 deg out of phase.
    The 0.10/0.20 defaults put them ~10x apart.
"""


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PositionController:
    """Horizontal position + velocity error -> commanded tilt, in radians.

    Returns (pitch_des, yaw_des) in the CURRENT repo convention: pitch about
    body x, yaw about body y. Renamed with everything else in a later commit
    (docs/CONVENTIONS.md).
    """

    def __init__(self, gains):
        self.gains = gains

    def update(self, pos_i, vel_i, pos_des):
        """
        pos_i, vel_i : inertial position [m] and velocity [m/s], (x, y, z)
        pos_des      : inertial target, (x, y, z); z is ignored (altitude loop)
        """
        ex = pos_i[0] - pos_des[0]
        ey = pos_i[1] - pos_des[1]
        g = self.gains
        lim = g.max_tilt

        # Sign derivation, preserved verbatim from sim/hover.py because both
        # signs were wrong once and the comment is what caught it:
        #
        #   Rotating body +z into the world: +yaw tips thrust toward +x, and
        #   +pitch tips it toward -y. So to come BACK to the origin the demands
        #   carry opposite signs to each other:
        #       x > 0  needs -x force -> negative yaw
        #       y > 0  needs -y force -> positive pitch
        #   Getting either backwards turns this loop into positive feedback and
        #   the vehicle accelerates away instead of returning.
        yaw_des = _clamp(-(g.kp_pos * ex + g.kd_pos * vel_i[0]), -lim, lim)
        pitch_des = _clamp(+(g.kp_pos * ey + g.kd_pos * vel_i[1]), -lim, lim)
        return pitch_des, yaw_des
