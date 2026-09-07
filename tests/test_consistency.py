"""
The numbers that live in two places must agree.
================================================================================
Every check here guards a duplicate that already caused a real bug once. The
pattern is always the same: a value gets copied, one copy is updated, nothing
errors, and the simulator quietly describes a different vehicle than the one the
controller was tuned for.

    the Gazebo model vs the YAML .......... the composite CG once landed at
                                            161 mm while everything else assumed
                                            211 mm
    the plugin ceiling vs the vehicle ..... 2*k*wmax^2 and thrust_at_max_n were
                                            independently edited fields that
                                            happened to agree to 0.02%
    the sign of tau_P ..................... three independent choices that agree
                                            by coincidence
    the two analytic entry paths .......... one modelling a lag the other does
                                            not is an unattributable difference

The fix for a duplicate is usually to generate one side from the other. Where
that is done, the test is that the generator's output matches the file on disk.
"""
import os
import re
import pytest

# --- solver constants vs physical limits -------------------------------------

def test_solver_constants_are_headroom_not_physics(vp, vehicle):
    """max_rot_velocity must EXCEED what the real vehicle can produce.

    It used to EQUAL it -- 2*k*wmax^2 was 17.787 N against a measured 17.79 --
    and the standalone hover controller calibrated thrust->omega from that
    coincidence while the plugin used motor_constant. The two are deliberately
    different now, so the old equality would be a bug and the inequality is the
    invariant.
    """
    rot = vehicle.raw["rotors"]
    ceiling = 2.0 * rot["motor_constant"] * rot["max_rot_velocity"] ** 2
    assert ceiling > vp.T_max * 1.1, \
        "no solver headroom: plugin ceiling %.2f N vs vehicle %.2f N" \
        % (ceiling, vp.T_max)


def test_the_measured_surface_and_thrust_at_max_agree(vp, vehicle):
    """thrust_at_max_n is quoted separately from the surface it came from."""
    T, _ = vp.surface.forward_norm(1.0, 1.0)
    assert T == pytest.approx(vehicle.raw["rotors"]["thrust_at_max_n"], abs=0.01)


def test_gimbal_nominal_limit_bounds_the_measured_travel(vp):
    """gimbal.max_deg is the symmetric summary the SDF joints use; every
    measured per-ring travel must fit inside it, or Gazebo's stops are tighter
    than the allocator's."""
    for lo, hi in zip(vp.delta_min, vp.delta_max):
        assert abs(lo) <= vp.gimbal_max + 1e-9
        assert abs(hi) <= vp.gimbal_max + 1e-9


def test_the_slowest_measured_ring_sets_the_nominal_slew(vp):
    """gimbal.rate_max_deg is a worst-case summary. If it were the faster ring,
    any consumer using it would let the outer ring move faster than it can."""
    assert vp.gimbal_rate_max <= min(vp.delta_rate_max) + 1e-9


# --- shared architecture ------------------------------------------------------

def test_both_analytic_paths_use_the_same_actuator_chain(repo):
    """The analytic harness and the ROS simulator node must not model different
    subsets of the actuator dynamics -- a difference there is an unattributable
    difference in every cross-plant comparison."""
    base = os.path.join(repo, "src", "tvc_control", "tvc_control")
    for path in (os.path.join(base, "harness", "mil.py"),
                 os.path.join(base, "nodes", "simulator.py")):
        src = open(path, encoding="utf-8").read()
        assert "ActuatorChain" in src, "%s builds its own actuator model" % path


def test_only_one_controller_implementation_exists(repo):
    """The whole point of the layer split.

    Every pipeline must reach the control law through TvcController. A file that
    computes a rate demand or a thrust command itself is a second controller,
    and this repository had exactly that problem for a long time: the Gazebo
    demo flew one implementation while the tests exercised another.
    """
    base = os.path.join(repo, "src", "tvc_control", "tvc_control")
    entry_points = [
        os.path.join(base, "harness", "mil.py"),
        os.path.join(base, "harness", "gz.py"),
        os.path.join(base, "nodes", "controller.py"),
    ]
    for path in entry_points:
        src = open(path, encoding="utf-8").read()
        assert "TvcController" in src, \
            "%s does not go through TvcController" % os.path.basename(path)

    # And no gain constant may be defined outside the gains file.
    offenders = []
    for dirpath, dirnames, filenames in os.walk(base):
        dirnames[:] = [d for d in dirnames if d != "__pycache__"]
        for name in filenames:
            if not name.endswith(".py") or name == "params.py":
                continue
            path = os.path.join(dirpath, name)
            for n, line in enumerate(open(path, encoding="utf-8"), 1):
                code = line.split("#")[0]
                for token in ("KP_", "KD_", "KI_"):
                    if token in code and "=" in code:
                        offenders.append("%s:%d %s" % (name, n, line.strip()[:70]))
    assert not offenders, ("control gains defined in code rather than in "
                           "control_gains.yaml:\n  " + "\n  ".join(offenders))


# --- the sign of tau_P, end to end -------------------------------------------

def test_tau_p_sign_chain_agrees_end_to_end(vp, vehicle):
    """Surface, allocator and Gazebo plugin must mean the same thing by +tau_P.

    Commanding MORE on rotor B than rotor A must give POSITIVE roll torque in
    all three. The surface says so through its gradients (dTz/db > 0 > dTz/da);
    the plugin says so through tau = c*(T_b - T_a); the allocator inherits it.
    Nothing forced these three independent choices to agree -- they do by
    coincidence, so it is asserted rather than trusted. If one flips, the vehicle
    spins up instead of correcting and it reads as a control bug.
    """
    from tvc_control.gnc.allocation import allocate
    from tvc_control.hal.gazebo import plugin_forward, rotor_speeds

    rot = vehicle.raw["rotors"]
    k, c, wmax = (rot["motor_constant"], rot["moment_constant"],
                  rot["max_rot_velocity"])

    # 1. the measured surface: more B than A -> positive
    _, q_more_b = vp.surface.forward_norm(-1.0, +1.0)
    _, q_more_a = vp.surface.forward_norm(+1.0, -1.0)
    assert q_more_b > 0.0 > q_more_a

    # 2. the plugin, driven by the inversion: a positive tau_P request must put
    #    the faster rotor on B
    wa, wb = rotor_speeds(13.0, +0.05, k, c, wmax)
    assert wb > wa
    _, q = plugin_forward(wa, wb, k, c)
    assert q > 0.0

    # 3. and the allocator agrees about which way M_z points
    assert allocate((0.0, 0.0, +0.05), 13.0, vp).tau_p > 0.0


def test_the_two_worlds_agree_on_physics_and_plugins(repo):
    """tvc.sdf and tvc_flight.sdf duplicate ~50 lines of preamble.

    SDF has no include mechanism for world-level settings, so the duplication is
    forced. What is not forced is letting the two copies drift: a different
    solver step in one world would make results from it incomparable with the
    other, silently. The vehicle spawn and the chase camera are SUPPOSED to
    differ -- that is what the two worlds are for -- so only the shared parts
    are compared.
    """
    worlds = os.path.join(repo, "gazebo", "worlds")
    a = open(os.path.join(worlds, "tvc.sdf"), encoding="utf-8").read()
    b = open(os.path.join(worlds, "tvc_flight.sdf"), encoding="utf-8").read()

    for tag in ("max_step_size", "real_time_factor"):
        va = re.search(r"<%s>([^<]+)</%s>" % (tag, tag), a)
        vb = re.search(r"<%s>([^<]+)</%s>" % (tag, tag), b)
        assert va and vb, "neither world declares <%s>" % tag
        assert va.group(1) == vb.group(1), \
            "the two worlds disagree about <%s>: %s vs %s" \
            % (tag, va.group(1), vb.group(1))

    def systems(text):
        # the chase camera is deliberately only in tvc_flight
        return set(re.findall(r'<plugin filename="(gz-sim-[a-z-]+)"', text))

    only_flight = systems(b) - systems(a)
    only_ground = systems(a) - systems(b)
    assert not only_ground, \
        "tvc.sdf loads system plugins tvc_flight.sdf does not: %s" % sorted(only_ground)
    assert only_flight <= {"gz-sim-sensors-system"}, \
        "unexpected extra system plugins in tvc_flight.sdf: %s" % sorted(only_flight)
