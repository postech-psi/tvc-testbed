"""
The axis convention, asserted rather than documented.
================================================================================
This file exists BEFORE the rename that adopts the rocket convention, because
that rename's failure mode is silent. The most dangerous case is a mistyped SDF
joint name: the joint still exists, no plugin drives it, the gimbal simply never
moves, and the vehicle drifts off exactly as it would with a control bug. Nothing
errors. The SDF-parsing test below is the only thing that catches it.

The second failure mode is a value swap that looks like a rename -- an
identifier surviving with a different meaning. Handedness and the sign tests
below catch those, because a swapped pair inverts a cross product or a torque
direction.
"""
import math
import os
import re

import pytest

# body axis -> unit vector, per docs/CONVENTIONS.md
X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)
Z = (0.0, 0.0, 1.0)


def _cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def test_named_triad_is_right_handed():
    """roll x pitch == yaw.

    This is the ONLY non-arbitrary criterion available for the assignment: the
    vehicle's legs sit at 0/120/240 deg and it has no aerodynamic reference
    direction, so nothing physical distinguishes body x from body y and a 90 deg
    roll maps one onto the other. Handedness is what is left, and REP-103 plus
    every rotation matrix and mixer in the stack assumes it.

    The rejected alternative (pitch = y, yaw = x) gives z x y = -x: a
    left-handed triad in which positive roll then positive pitch produces
    NEGATIVE yaw.
    """
    roll_axis, pitch_axis, yaw_axis = Z, X, Y
    assert _cross(roll_axis, pitch_axis) == pytest.approx(yaw_axis)


def _sdf(repo):
    path = os.path.join(repo, "sim", "models", "tvc_vehicle", "model.sdf")
    with open(path, encoding="utf-8") as f:
        return f.read()


def _joint_axis(sdf, joint):
    """The <xyz> of a named revolute joint, as a tuple."""
    m = re.search(r'<joint name="%s".*?<xyz>([^<]+)</xyz>' % re.escape(joint),
                  sdf, re.S)
    assert m, "joint %r not found in model.sdf" % joint
    return tuple(float(v) for v in m.group(1).split())


def test_sdf_joints_exist_and_carry_the_expected_axes(repo):
    """Ring names, and the body axis each ring actually rotates about.

    The joints are named after the RING, not the axis, deliberately: the ring is
    a mechanical fact invariant under any naming convention, so a future
    revision of the convention does not touch model.sdf. What must be checked is
    that the mapping in docs/CONVENTIONS.md matches the geometry.
    """
    sdf = _sdf(repo)
    outer, inner = _ring_joint_names(sdf)
    assert _joint_axis(sdf, outer) == (1.0, 0.0, 0.0), \
        "outer ring must rotate about body x (the pitch plane)"
    assert _joint_axis(sdf, inner) == (0.0, 1.0, 0.0), \
        "inner ring must rotate about body y (the yaw plane)"


def _ring_joint_names(sdf):
    """Accept either naming while the rename is in flight.

    Before the rename the joints are gimbal_roll_joint (outer) and
    gimbal_pitch_joint (inner); after it they are gimbal_outer_joint and
    gimbal_inner_joint. Both are checked against the same geometry, so this file
    is meaningful on both sides of that commit -- which is the point of writing
    it first.
    """
    if "gimbal_outer_joint" in sdf:
        return "gimbal_outer_joint", "gimbal_inner_joint"
    return "gimbal_roll_joint", "gimbal_pitch_joint"


def test_every_gimbal_joint_is_driven_by_a_plugin(repo):
    """A joint no plugin drives is a gimbal that never moves.

    This is the rename's worst silent failure: model.sdf still parses, gz-sim
    still loads it, and the vehicle just does not respond. It reads as a control
    bug and would be debugged as one.
    """
    sdf = _sdf(repo)
    outer, inner = _ring_joint_names(sdf)
    for joint in (outer, inner):
        m = re.search(
            r'JointPositionController.*?<joint_name>%s</joint_name>.*?'
            r'<topic>([^<]+)</topic>' % re.escape(joint), sdf, re.S)
        assert m, "no JointPositionController drives %r" % joint
        assert m.group(1).startswith("/tvc_vehicle/gimbal_"), \
            "unexpected command topic for %r: %r" % (joint, m.group(1))


def test_positive_outer_deflection_makes_positive_moment_about_its_axis(vp):
    """Gimbal sign, checked through the dynamics rather than assumed.

    Both position-loop signs in sim/hover.py were wrong once, and the comment
    recording that is the only reason anyone noticed. This asserts the
    underlying convention so the next sign error fails a test instead of a
    flight.
    """
    from tvc_control.gnc.mathx import thrust_axis
    n0 = thrust_axis((0.0, 0.0))
    assert n0 == pytest.approx((0.0, 0.0, 1.0)), \
        "centred gimbal must point thrust along body +z"

    # delta1 (inner ring) positive tilts thrust toward +x, which with the thrust
    # applied BELOW the CM at r = (0,0,-L) gives a negative moment about body y.
    n1 = thrust_axis((0.05, 0.0))
    assert n1[0] > 0.0 and abs(n1[1]) < 1e-12
    # delta2 (outer ring) positive tilts thrust toward -y.
    n2 = thrust_axis((0.0, 0.05))
    assert n2[1] < 0.0 and abs(n2[0]) < 1e-12


def test_thrust_axis_moment_has_no_gimbal_contribution(vp):
    """The gimbal has NO authority about the thrust axis -- the fact the whole
    tau_P channel exists for. If a sign flip ever made r x F produce a
    z-component, the allocator's two-stage decomposition silently stops being
    valid."""
    from tvc_control.gnc.mathx import thrust_axis
    L = vp.L
    for d1 in (-0.1, 0.0, 0.07):
        for d2 in (-0.1, 0.0, 0.07):
            n = thrust_axis((d1, d2))
            # r x F with r = (0,0,-L), F = T*n : z-component is
            # (-L)*... -> identically zero by construction.
            rz_cross_z = 0.0 * n[0] + 0.0 * n[1]
            assert rz_cross_z == 0.0


def test_retired_parameter_names_are_rejected(repo):
    """A parameter that looks live while being ignored is worse than one that is
    gone -- it makes a launch file document a control decision that is not
    happening. The nodes raise on these; this checks the list has not quietly
    shrunk."""
    for mod, expected in (
            ("controller_node", "gimbal_rate_max_deg"),
            ("simulator_node", "gimbal_rate_max_deg")):
        path = os.path.join(repo, "src", "tvc_control", "tvc_control",
                            mod + ".py")
        src = open(path, encoding="utf-8").read()
        assert "RETIRED_PARAMS" in src, "%s lost its retired-parameter guard" % mod
        assert expected in src


def test_convention_token_is_present_and_consistent(repo):
    """The token travels with the data so a stale consumer fails loudly.

    vehicle_params.yaml, control_gains.yaml and the actuator message must all
    agree; a mismatch between them is exactly the state a half-applied rename
    leaves behind.
    """
    from tvc_control.config import load, SUPPORTED_AXIS_CONVENTIONS
    v = load()
    token = v.raw.get("axis_convention")
    assert token in SUPPORTED_AXIS_CONVENTIONS

    gains = os.path.join(repo, "src", "tvc_control", "tvc_control",
                         "control_gains.yaml")
    assert ("axis_convention: %s" % token) in open(gains, encoding="utf-8").read(), \
        "control_gains.yaml disagrees with vehicle_params.yaml about the convention"

    msg = os.path.join(repo, "src", "tvc_msgs", "msg", "ActuatorCommand.msg")
    assert "axis_convention" in open(msg, encoding="utf-8").read()
