"""
The Gazebo gimbal servo must be stable at the world's physics step.
================================================================================
This file exists because of one bug, and it is worth stating plainly.

The SDF's JointPositionController gains were hand-written as p=60, d=1.0 on a
gimbal ring whose reflected inertia is 8.5e-5 kg.m^2, at a 1 ms physics step.
JointPositionController is an explicit PID writing a joint force each step, so
the damping term integrates as

    omega <- omega * (1 - d*dt/I)

and d*dt/I was 11.8. That diverges. The joint chattered against its +/-5 N.m
clamp, the reaction torque went into the airframe, and the vehicle tumbled to
180 degrees in about a second -- with the gimbal commanded to exactly zero and
the controller not running at all.

Nothing caught it because nothing here knew the servo had a stability
condition. Every check below is that condition, written down.

The numbers come from gen_model_sdf.py, which derives the gains from each
ring's own inertia, and from the world files, which own the step. The test
reads BOTH off disk rather than importing the generator's opinion of them, so a
hand edit to model.sdf or a coarser step in a world file fails here.
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))


@pytest.fixture(scope="module")
def servos(repo):
    """-> {ring: (p_gain, d_gain, cmd_max)} read out of the generated SDF."""
    path = os.path.join(repo, "gazebo", "models", "tvc_vehicle", "model.sdf")
    root = ET.parse(path).getroot()
    out = {}
    for plugin in root.iter("plugin"):
        if "JointPositionController" not in (plugin.get("name") or ""):
            continue
        joint = plugin.findtext("joint_name")
        ring = "outer" if "outer" in joint else "inner"
        out[ring] = (float(plugin.findtext("p_gain")),
                     float(plugin.findtext("d_gain")),
                     float(plugin.findtext("cmd_max")))
    return out


@pytest.fixture(scope="module")
def world_steps(repo):
    """-> {world filename: max_step_size} for every world that loads the model."""
    d = os.path.join(repo, "gazebo", "worlds")
    steps = {}
    for name in sorted(os.listdir(d)):
        if not name.endswith(".sdf"):
            continue
        text = open(os.path.join(d, name), encoding="utf-8").read()
        m = re.search(r"<max_step_size>([^<]+)</max_step_size>", text)
        if m:
            steps[name] = float(m.group(1))
    return steps


def test_both_rings_have_a_servo(servos):
    assert set(servos) == {"inner", "outer"}, servos


def test_the_damping_term_cannot_diverge(servos, world_steps):
    """d*dt/I < 1: the condition the old gains missed by a factor of twelve.

    Below 1 the explicit damping update contracts monotonically. Between 1 and 2
    it alternates in sign while still contracting -- a ringing joint. At 2 and
    above it grows, which is what happened.
    """
    from gen_model_sdf import ring_inertia
    for ring, (_, d_gain, _) in servos.items():
        inertia = ring_inertia("x" if ring == "outer" else "y", ring)
        for world, dt in world_steps.items():
            ratio = d_gain * dt / inertia
            assert ratio < 1.0, (
                "%s ring in %s: d*dt/I = %.2f. At >=2 the joint velocity grows "
                "every step; the p=60/d=1.0 gains gave 11.8 and tumbled the "
                "vehicle. See gen_model_sdf.py's SERVO_* block."
                % (ring, world, ratio))


def test_the_spring_term_cannot_diverge(servos, world_steps):
    """p*dt^2/I < 4, the explicit-integration limit for the stiffness term.

    This one the old gains actually passed (0.7), which is why the failure
    looked like something other than a servo problem.
    """
    from gen_model_sdf import ring_inertia
    for ring, (p_gain, _, _) in servos.items():
        inertia = ring_inertia("x" if ring == "outer" else "y", ring)
        for world, dt in world_steps.items():
            ratio = p_gain * dt * dt / inertia
            assert ratio < 4.0, "%s ring in %s: p*dt^2/I = %.2f" % (ring, world, ratio)


def test_the_servo_is_faster_than_the_gimbal_it_stands_in_for(servos, vp):
    """It must not add dynamics of its own on top of plant/actuators.py.

    The real gimbal is modelled with a 30 ms transport delay and per-ring slew
    limits, in ONE place, so both plants agree. If Gazebo's servo settled at a
    comparable speed it would be double-counting -- which is exactly what the
    p=3, i=0.02, d=0.15 gains this file replaced were doing.
    """
    from gen_model_sdf import ring_inertia
    for ring, (p_gain, d_gain, _) in servos.items():
        inertia = ring_inertia("x" if ring == "outer" else "y", ring)
        wn = (p_gain / inertia) ** 0.5
        zeta = d_gain / (2.0 * (p_gain * inertia) ** 0.5)
        settle = 4.0 / (zeta * wn)
        assert settle < 0.5 * vp.gimbal_deadtime_s, (
            "%s ring settles in %.0f ms against a modelled %.0f ms transport "
            "delay -- close enough to be a second lag rather than a servo"
            % (ring, 1e3 * settle, 1e3 * vp.gimbal_deadtime_s))


def test_the_servo_never_needs_its_torque_clamp(servos, vp):
    """cmd_max must not bind in normal flight.

    A clamped servo output is a rectified error: the joint stops tracking and
    starts pushing a one-sided torque into the airframe. That is the mechanism
    that tumbled the vehicle, so the check is that the worst legitimate demand
    -- full travel plus full slew -- stays inside the clamp.
    """
    import math
    for ring, (p_gain, d_gain, cmd_max) in servos.items():
        i = 0 if ring == "inner" else 1
        travel = max(abs(vp.delta_min[i]), abs(vp.delta_max[i]))
        slew = math.radians(vp.gimbal_rate_max_deg)
        worst = p_gain * travel + d_gain * slew
        assert worst < cmd_max, (
            "%s ring can demand %.2f N.m of the %.2f N.m clamp"
            % (ring, worst, cmd_max))


def test_every_world_uses_the_step_the_gains_were_designed_for(world_steps):
    """A coarser step in one world would silently unstabilise the servo there.

    The gains are derived against gen_model_sdf.WORLD_STEP_S. Nothing in SDF
    links a model's plugin gains to the world's integrator, so this is the link.
    """
    from gen_model_sdf import WORLD_STEP_S
    for world, dt in world_steps.items():
        assert dt == WORLD_STEP_S, (
            "%s steps at %g s but the servo gains are derived for %g s. Either "
            "match the world or move WORLD_STEP_S and regenerate model.sdf."
            % (world, dt, WORLD_STEP_S))
