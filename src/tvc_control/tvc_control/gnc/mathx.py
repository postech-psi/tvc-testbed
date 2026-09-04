"""
Small fixed-size math: quaternions and the gimballed thrust direction.
================================================================================
Shared by the controller and the plant. Math is neither -- putting it in `gnc`
rather than duplicating it keeps one definition of the quaternion convention,
which is the kind of thing that silently forks and then disagrees by a sign.

Convention: quaternions are (qw, qx, qy, qz), body <- inertial, unit norm.
Euler angles are READOUT ONLY -- kinematics are never propagated through them,
which is what keeps the +/-90 deg singularity out of the state.
"""

import math


def quat_normalize(q):
    """Renormalize to unit quaternion (mandatory after every integration step;
    truncation error accumulates otherwise -- see Step 1 guide, Prop. on renorm)."""
    qw, qx, qy, qz = q
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (qw / n, qx / n, qy / n, qz / n)


def quat_to_rotmat(q):
    """Body -> inertial rotation matrix R(q), from unit quaternion q=(qw,qx,qy,qz).

    Returned as a tuple of rows. Callers that want to matrix-multiply wrap it in
    np.array -- but only the plant does that, because only the plant integrates.
    """
    qw, qx, qy, qz = q
    return (
        (1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)),
        (2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)),
        (2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)),
    )


def quat_to_euler(q):
    """ZYX (yaw-pitch-roll) Euler angles, for readout/plotting ONLY -- never
    used internally for kinematics propagation (that would reintroduce gimbal lock)."""
    qw, qx, qy, qz = q
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    s = 2*(qw*qy - qz*qx)
    s = 1.0 if s > 1.0 else (-1.0 if s < -1.0 else s)
    pitch = math.asin(s)
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return (roll, pitch, yaw)


def euler_to_quat(roll, pitch, yaw):
    """Construct a quaternion from ZYX Euler angles (used only to set up initial
    conditions / disturbances in a human-friendly way)."""
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy)


def thrust_axis(delta):
    """Unit vector n_hat along the gimballed thrust axis, in the body frame.

    Exact (not small-angle). Sign convention matches Gazebo / sim/hover.py --
    n_hat = R_x(d2) R_y(d1) [0,0,1] -- so gimbal signs and gains are portable
    between the two sims. d1 deflects in the pitch plane (about body y),
    d2 in the roll plane (about body x).
    """
    d1, d2 = delta[0], delta[1]
    return (math.sin(d1),
            -math.sin(d2) * math.cos(d1),
            math.cos(d1) * math.cos(d2))


def clamp(v, lo, hi):
    """The one clamp. numpy's clip is not available in flight code."""
    return lo if v < lo else (hi if v > hi else v)


def isclose(a, b):
    """numpy.isclose's default predicate, reproduced exactly.

    Written out rather than approximated because it decides saturation flags,
    and a flag that fires one step earlier or later changes anti-windup
    behaviour. rtol=1e-5, atol=1e-8, compared against |b|.
    """
    return abs(a - b) <= 1e-8 + 1e-5 * abs(b)


def quat_mul(p, q):
    """Hamilton product p (x) q, both (qw, qx, qy, qz)."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (pw*qw - px*qx - py*qy - pz*qz,
            pw*qx + px*qw + py*qz - pz*qy,
            pw*qy - px*qz + py*qw + pz*qx,
            pw*qz + px*qy - py*qx + pz*qw)


def quat_conj(q):
    """Conjugate = inverse for a unit quaternion."""
    return (q[0], -q[1], -q[2], -q[3])


def attitude_error(q, q_des):
    """Body-frame attitude error vector [rad], sign-matched to (desired - actual).

    Returns 2*sgn(qe_w)*qe_v for qe = q^-1 (x) q_des, which is the standard
    small-rotation error and equals the Euler difference to first order -- the
    equivalence is asserted in tests/test_attitude_error.py rather than argued.

    The sgn() picks the short way round. Without it the controller happily takes
    the 350 deg path to a 10 deg target, which on a vehicle with 7 deg of gimbal
    travel is not a slow recovery but a tumble.

    Why this replaces the Euler difference: the Euler path had an arcsin
    singularity at +/-90 deg on one specific axis, which meant the axis names
    carried a stability caveat. Under the rocket convention that axis becomes
    "yaw", and a reader would have to know which of three similar-looking
    channels was the fragile one. This form has no preferred axis at all.
    """
    qe = quat_mul(quat_conj(q), q_des)
    s = 1.0 if qe[0] >= 0.0 else -1.0
    return (2.0 * s * qe[1], 2.0 * s * qe[2], 2.0 * s * qe[3])
