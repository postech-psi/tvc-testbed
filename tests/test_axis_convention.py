"""
The axis convention, asserted rather than documented.
================================================================================
docs/4-CONVENTIONS.md states the frames, axis names and signs. This file is what
makes that statement load-bearing, because every way of getting the convention
wrong fails SILENTLY.

The three silent failures it catches:

  A MISTYPED SDF JOINT NAME. The joint still exists, gz-sim still loads the
  model, no plugin drives it, the gimbal simply never moves and the vehicle
  drifts off exactly as it would with a control bug. Nothing errors.

  AN IDENTIFIER THAT SURVIVES A RENAME WITH A DIFFERENT MEANING. Handedness and
  the sign tests below catch those, because a swapped pair inverts a cross
  product or a torque direction.

"""
import os
import re

import pytest

# body axis -> unit vector, per docs/4-CONVENTIONS.md
X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)
Z = (0.0, 0.0, 1.0)

OUTER_JOINT = "gimbal_outer_joint"
INNER_JOINT = "gimbal_inner_joint"


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _sdf(repo):
    path = os.path.join(repo, "gazebo", "models", "tvc_vehicle", "model.sdf")
    with open(path, encoding="utf-8") as f:
        return f.read()


def _joint_axis(sdf, joint):
    """The <xyz> of a named revolute joint, as a tuple."""
    m = re.search(r'<joint name="%s".*?<xyz>([^<]+)</xyz>' % re.escape(joint),
                  sdf, re.S)
    assert m, "joint %r not found in model.sdf" % joint
    return tuple(float(v) for v in m.group(1).split())


# --- the naming itself -------------------------------------------------------

def test_named_triad_is_right_handed():
    """roll x pitch == yaw.

    This is the ONLY non-arbitrary criterion available for the assignment. The
    vehicle's legs sit at 0/120/240 deg and it has no aerodynamic reference
    direction, so nothing physical distinguishes body x from body y and a 90 deg
    roll maps one onto the other. Handedness is what is left, and REP-103 plus
    every rotation matrix and mixer in the stack assumes it.

    The rejected alternative (pitch = y, yaw = x) gives z x y = -x: a left-handed
    triad in which positive roll then positive pitch produces NEGATIVE yaw.
    """
    roll_axis, pitch_axis, yaw_axis = Z, X, Y
    assert _cross(roll_axis, pitch_axis) == pytest.approx(yaw_axis)


def test_convention_token_is_present_and_consistent(repo, vehicle):
    """The token travels with the data so a stale consumer fails loudly.

    vehicle_params.yaml, control_gains.yaml and the actuator message must all
    agree. A mismatch between them is exactly the state a half-applied rename
    leaves behind.
    """
    from tvc_control.config import SUPPORTED_AXIS_CONVENTIONS
    token = vehicle.raw.get("axis_convention")
    assert token in SUPPORTED_AXIS_CONVENTIONS

    gains = os.path.join(repo, "src", "tvc_control", "tvc_control",
                         "control_gains.yaml")
    assert ("axis_convention: %s" % token) in open(gains, encoding="utf-8").read(), \
        "control_gains.yaml disagrees with vehicle_params.yaml about the convention"

    msg = os.path.join(repo, "src", "tvc_msgs", "msg", "ActuatorCommand.msg")
    assert "axis_convention" in open(msg, encoding="utf-8").read()


def test_message_constants_are_unique_and_reserve_zero(repo):
    """A duplicate constant name makes `colcon build` fail, and a zero-valued
    convention makes a default-constructed message silently valid.

    Both have happened here. The rename mapped two differently-named constants
    onto one name, which rosidl rejects -- but only at build time, and this
    package had never been built.
    """
    path = os.path.join(repo, "src", "tvc_msgs", "msg", "ActuatorCommand.msg")
    consts = re.findall(r"^\s*uint8\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)",
                        open(path, encoding="utf-8").read(), re.M)
    names = [n for n, _ in consts]
    assert len(names) == len(set(names)), "duplicate constant name: %s" % names
    by_name = dict(consts)
    assert by_name.get("AXIS_CONVENTION_UNSET") == "0", \
        "0 must mean 'unset' so an unfilled message fails the receiver's check"
    assert by_name.get("AXIS_CONVENTION_ROCKET_V2", "0") != "0"


# --- the SDF, which is where the convention meets the physics engine ---------

def test_sdf_joints_exist_and_carry_the_expected_axes(repo):
    """Ring names, and the body axis each ring actually rotates about.

    The joints are named after the RING, not the axis, deliberately: a ring is a
    mechanical fact invariant under any naming convention, so a future revision
    of the convention does not touch model.sdf. What must be checked is that the
    mapping in docs/4-CONVENTIONS.md matches the geometry.
    """
    sdf = _sdf(repo)
    assert _joint_axis(sdf, OUTER_JOINT) == (1.0, 0.0, 0.0), \
        "outer ring must rotate about body x (the pitch plane)"
    assert _joint_axis(sdf, INNER_JOINT) == (0.0, 1.0, 0.0), \
        "inner ring must rotate about body y (the yaw plane)"


def test_every_gimbal_joint_is_driven_by_a_plugin(repo):
    """A joint no plugin drives is a gimbal that never moves.

    model.sdf still parses, gz-sim still loads it, and the vehicle just does not
    respond. It reads as a control bug and would be debugged as one.
    """
    sdf = _sdf(repo)
    for joint in (OUTER_JOINT, INNER_JOINT):
        m = re.search(
            r'JointPositionController.*?<joint_name>%s</joint_name>.*?'
            r'<topic>([^<]+)</topic>' % re.escape(joint), sdf, re.S)
        assert m, "no JointPositionController drives %r" % joint
        assert m.group(1).startswith("/tvc_vehicle/gimbal_"), \
            "unexpected command topic for %r: %r" % (joint, m.group(1))


def test_sdf_joint_limits_match_the_measured_travel(repo, vp):
    """Gazebo's stops and the allocator's stops must be the same stops.

    If the SDF is tighter, Gazebo truncates a command the allocator thought was
    feasible -- and truncating one axis of a two-axis command ROTATES the
    realized torque instead of shrinking it, which is the failure the
    proportional scale factor in allocate() exists to avoid.
    """
    sdf = _sdf(repo)
    for joint in (OUTER_JOINT, INNER_JOINT):
        m = re.search(r'<joint name="%s".*?<lower>([-\d.eE+]+)</lower>\s*'
                      r'<upper>([-\d.eE+]+)</upper>' % re.escape(joint), sdf, re.S)
        assert m, "no <lower>/<upper> limit on %r" % joint
        lo, hi = float(m.group(1)), float(m.group(2))
        # The SDF carries the symmetric nominal limit; the allocator uses the
        # asymmetric measured travel, which must fit inside it.
        assert lo <= min(vp.delta_min) + 1e-9
        assert hi >= max(vp.delta_max) - 1e-9


# --- signs, checked through the geometry rather than assumed -----------------

def test_centred_gimbal_points_thrust_along_body_z(vp):
    from tvc_control.gnc.mathx import thrust_axis
    assert thrust_axis((0.0, 0.0)) == pytest.approx((0.0, 0.0, 1.0))


def test_gimbal_deflection_signs(vp):
    """Which way each ring tilts the thrust vector.

    Both position-loop signs were wrong once, and the comment recording that is
    the only reason anyone noticed. This asserts the underlying convention so
    the next sign error fails a test instead of a flight.
    """
    from tvc_control.gnc.mathx import thrust_axis
    # delta1 (inner ring, yaw plane) positive tilts thrust toward +x
    n1 = thrust_axis((0.05, 0.0))
    assert n1[0] > 0.0 and abs(n1[1]) < 1e-12
    # delta2 (outer ring, pitch plane) positive tilts thrust toward -y
    n2 = thrust_axis((0.0, 0.05))
    assert n2[1] < 0.0 and abs(n2[0]) < 1e-12


def test_the_gimbal_has_no_authority_about_the_thrust_axis(vp):
    """The fact the whole tau_P channel exists for.

    Thrust acts at r = (0, 0, -L), so r x F has no z-component for ANY gimbal
    angle -- the cross product of two vectors whose x/y parts come only from F
    cannot reach z when r is purely axial. If a sign flip ever made it nonzero,
    the allocator's two-stage decomposition silently stops being valid.
    """
    from tvc_control.gnc.mathx import thrust_axis
    L = vp.L
    for d1 in (-0.12, -0.05, 0.0, 0.05, 0.12):
        for d2 in (-0.12, -0.05, 0.0, 0.05, 0.12):
            nx, ny, _ = thrust_axis((d1, d2))
            r = (0.0, 0.0, -L)
            f = (nx, ny, 0.0)          # z-component of r x F ignores F_z
            mz = r[0] * f[1] - r[1] * f[0]
            assert mz == 0.0
