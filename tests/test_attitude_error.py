"""
The quaternion attitude error, and why it replaced the Euler difference.
================================================================================
gnc/mathx.py claims two things about `attitude_error`, and both are asserted
here rather than argued in prose:

  1. IT AGREES WITH THE EULER DIFFERENCE TO FIRST ORDER. That is what makes the
     change safe -- the gains keep their meaning, because at the few degrees
     this vehicle flies at the two forms are the same number.

  2. IT TAKES THE SHORT WAY ROUND. Without the sgn(qe_w) factor a controller
     happily takes the 350 degree path to a 10 degree target, which on a vehicle
     with 7 degrees of gimbal travel is not a slow recovery but a tumble.

The reason for the change was the Euler path's arcsin singularity at +/-90 deg
on one specific axis, which meant the axis NAMES carried a stability caveat.
Under the rocket convention that axis is called "roll", and a reader would have
to know which of three similar-looking channels was the fragile one. The
quaternion form has no preferred axis, so the caveat disappears instead of
moving -- which is what the last test here checks.
"""
import math

import pytest

from tvc_control.gnc.mathx import (attitude_error, euler_to_quat, quat_to_euler)


def test_zero_error_at_the_setpoint():
    q = euler_to_quat(0.1, -0.2, 0.05)
    assert attitude_error(q, q) == pytest.approx((0.0, 0.0, 0.0), abs=1e-15)


@pytest.mark.parametrize("deg", [0.0, 1.0, 2.0, 5.0, 8.0, 15.0])
def test_agrees_with_the_euler_difference_to_second_order(deg):
    """|quaternion error - Euler difference| <= 0.21 * |theta|^2, measured.

    This is the compatibility claim the switch rested on, stated as a bound
    rather than a tolerance someone tuned until it passed. The deviation is
    second order because the two forms differ only in how a composition of three
    rotations is linearized, so it is 0.1% of the angle at 1 deg and 4.7% at the
    8 deg the position loop is allowed to command. The gains kept their meaning
    because at those angles the error signal did not materially change.
    """
    a = math.radians(deg)
    ang = (a, -0.7 * a, 0.5 * a)          # a generic direction, not axis-aligned
    e = attitude_error(euler_to_quat(0.0, 0.0, 0.0), euler_to_quat(*ang))
    theta = math.sqrt(sum(v * v for v in ang))
    worst = max(abs(x - y) for x, y in zip(e, ang))
    assert worst <= 1e-15 + 0.21 * theta ** 2, \
        "deviation %.3e at |theta| = %.4f rad is worse than second order" \
        % (worst, theta)


def test_takes_the_short_way_round():
    """A target 10 deg away must give a 10 deg error, not a 350 deg one."""
    small = math.radians(10.0)
    q = euler_to_quat(0.0, 0.0, 0.0)
    near = euler_to_quat(0.0, 0.0, small)
    far = euler_to_quat(0.0, 0.0, small - 2.0 * math.pi)   # the same attitude
    e_near, e_far = attitude_error(q, near), attitude_error(q, far)
    # Physically identical attitudes must give identical errors...
    assert e_near == pytest.approx(e_far, abs=1e-12)
    # ...and that error must be the short one.
    assert abs(e_near[2]) < math.radians(15.0)


def test_no_axis_is_singular():
    """The property the Euler form did not have.

    Sweep each axis through +/-89 degrees -- the neighbourhood where the ZYX
    arcsin readout degenerates -- and require a finite, correctly-signed error
    on every one. No axis may be more fragile than the others.
    """
    for axis in range(3):
        for deg in (-89.0, -60.0, -1.0, 1.0, 60.0, 89.0):
            ang = [0.0, 0.0, 0.0]
            ang[axis] = math.radians(deg)
            e = attitude_error(euler_to_quat(0.0, 0.0, 0.0), euler_to_quat(*ang))
            assert all(math.isfinite(v) for v in e)
            assert e[axis] * math.radians(deg) > 0.0, \
                "error sign flipped on axis %d at %.0f deg" % (axis, deg)


def test_euler_readout_round_trips():
    """quat_to_euler is READOUT ONLY, but it still has to be right -- every plot,
    every metric and every log column goes through it."""
    for ang in ((0.0, 0.0, 0.0), (0.1, -0.2, 0.3), (-0.4, 0.15, -0.25)):
        out = quat_to_euler(euler_to_quat(*ang))
        assert out == pytest.approx(ang, abs=1e-12)


# --- the body/inertial boundary ----------------------------------------------
# nav_msgs/Odometry and gz.msgs.Odometry both report the twist in the CHILD
# (body) frame, per REP-105. EstimatedState.vel_i is inertial. Three call sites
# now convert -- harness/gz.py, nodes/controller.py, nodes/simulator.py -- and
# for a long time none of them did, which is exact only while the vehicle is
# level. A TVC test is never level.

def test_rotating_body_to_inertial_and_back_is_the_identity():
    import math
    from tvc_control.gnc.mathx import euler_to_quat, quat_rotate, quat_rotate_inv
    q = euler_to_quat(math.radians(10), math.radians(-7), math.radians(35))
    for v in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
              (0.3, -1.7, 4.2)):
        back = quat_rotate_inv(q, quat_rotate(q, v))
        for a, b in zip(v, back):
            assert abs(a - b) < 1e-12


def test_rotation_agrees_with_the_matrix_the_plant_integrates():
    """quat_rotate must be R(q) @ v and not something merely similar.

    The plant builds the matrix and multiplies with numpy; flight code cannot.
    Two implementations of one rotation is exactly the kind of thing that forks
    by a transpose, so they are compared rather than trusted.
    """
    import math
    import numpy as np
    from tvc_control.gnc.mathx import euler_to_quat, quat_rotate, quat_to_rotmat
    q = euler_to_quat(math.radians(12), math.radians(-25), math.radians(3))
    R = np.array(quat_to_rotmat(q))
    v = np.array([0.3, -1.7, 4.2])
    assert np.allclose(np.array(quat_rotate(q, tuple(v))), R @ v, atol=1e-12)


def test_a_free_falling_tilted_vehicle_shows_the_frames_are_different():
    """The measurement that found the bug, as an assertion.

    In free fall from the world's 10/-7 deg spawn the INERTIAL velocity is
    purely vertical, while the body-frame velocity Gazebo actually reports has
    a lateral component of |v|*sin(tilt). Reading one as the other is a silent
    lateral velocity error of 21% of the descent rate at this attitude.
    """
    import math
    from tvc_control.gnc.mathx import euler_to_quat, quat_rotate_inv
    from tvc_control.gnc.mathx import quat_to_rotmat
    q = euler_to_quat(math.radians(10.0), math.radians(-7.0), 0.0)
    speed = 3.295
    vx, vy, vz = quat_rotate_inv(q, (0.0, 0.0, -speed))

    # Exact, not hypot(10, 7): two Euler rotations do not add as a right
    # triangle. The true tilt is acos(R_zz), and hypot only happens to be close.
    tilt = math.acos(quat_to_rotmat(q)[2][2])
    assert abs(math.hypot(vx, vy) - speed * math.sin(tilt)) < 1e-9
    assert abs(abs(vz) - speed * math.cos(tilt)) < 1e-9
    # 0.696 m/s of lateral velocity that is not there -- 21% of the descent
    # rate, and exactly what Gazebo reported in the free-fall probe.
    assert math.hypot(vx, vy) > 0.6
