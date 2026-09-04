"""
Gazebo HAL: ActuatorSetpoint -> what gz-sim's plugins consume.
================================================================================
A HAL is neither flight code nor plant. It translates the controller's output
into one transport's units and does no control of its own. This one has a single
interesting function, and it is the answer to a question the linear plugin model
cannot otherwise answer.

THE PROBLEM
    gz-sim's MulticopterMotorModel computes, per rotor,
        T_i = k * omega_i^2 ,  tau_i = +/- c * T_i
    hence T = k(omega_a^2 + omega_b^2) and tau_P = c(T_b - T_a). That model is
    separable, symmetric and linear in the thrust split. The measured coax
    surface is none of the three, and the gap is not small: at hover the plugin
    with the original c = 0.016 reaches only ~50% of the measured roll torque,
    and being odd in the split it cannot represent the sign asymmetry at all --
    the surface gives +0.147 one way and -0.089 the other.

WHY NOT JUST FIT c
    Because a single c does not exist. The value required to cover the measured
    envelope runs from 0.016 at low thrust to 0.054 near the ceiling, a 3.3x
    spread. And fitting the middle would over-promise the weak direction by ~65%
    at hover: the controller would command a torque the plant cannot produce,
    the roll integrator would wind up, and the symptom would look like a tuning
    problem rather than a modelling one. Per-rotor asymmetric constants are
    worse still -- unequal k means equal omega no longer gives equal thrust, so
    the vehicle produces a parasitic pitch torque at zero pitch command.

THE FIX: INVERT ON THE COMMAND SIDE
    Take the (T, tau_P) the allocator chose off the measured surface, and solve
    the plugin's OWN algebra for the omega pair that makes it produce exactly
    that. Two equations, two unknowns, closed form, exact. The asymmetry is
    fully reproduced because it lives in the commanded tau_P -- which came from
    the surface -- and not in the plugin.

WHAT THIS COSTS, STATED PLAINLY
    Rotor angular velocity in Gazebo stops being a physical RPM. It is a control
    allocation variable; momentConstant becomes a solver scaling constant and
    maxRotVelocity becomes solver headroom. Nothing may read omega as physics,
    and the raised ceiling means the plugin no longer enforces the vehicle's real
    17.79 N thrust limit -- the allocator does, and tests assert it rather than
    assuming it. See docs/7-CREDIBILITY.md, Results Robustness.

    Feasibility drove the SDF change. At the original maxRotVelocity = 1100 the
    per-rotor ceiling is 8.893 N, and the c required to cover the envelope
    diverges near full thrust, so no single value works. At 1200+ the requirement
    converges to 0.0323 (binding at a near-idle corner), so the model uses 1300
    and c = 0.04 for margin. Checked across the measured envelope: hover with
    tau_P = 0 gives (941, 941), the strong direction (798, 1066), the weak
    direction (1019, 857), full thrust (1100, 1100) -- all inside 1300.
"""

import math


def rotor_speeds(thrust_n, tau_p_nm, motor_constant, moment_constant,
                 max_rot_velocity):
    """(T, tau_P) -> (omega_a, omega_b) [rad/s] for gz.msgs.Actuators.

    rotor_a is the CCW rotor (motorNumber 0), rotor_b the CW one, matching the
    SDF. The sign convention -- tau_P = c*(T_b - T_a) -- is the plugin's, and it
    agrees with the measured surface's (dTz/db positive, dTz/da negative) and
    with the allocator's. That three-way agreement is a coincidence of
    independent choices, so tests/test_consistency.py asserts it end to end
    rather than trusting it.

    Returns speeds clamped to [0, max_rot_velocity]. A clamp here means the
    request was outside what the SOLVER can express, which is a different and
    much rarer thing than the vehicle being out of authority -- the allocator
    has already enforced the real feasible set upstream.
    """
    split = tau_p_nm / moment_constant
    t_a = 0.5 * (thrust_n - split)
    t_b = 0.5 * (thrust_n + split)
    wa = math.sqrt(max(t_a, 0.0) / motor_constant)
    wb = math.sqrt(max(t_b, 0.0) / motor_constant)
    return (min(wa, max_rot_velocity), min(wb, max_rot_velocity))


def plugin_forward(omega_a, omega_b, motor_constant, moment_constant):
    """The plugin's own model, for checking the inversion round-trips.

    Not used in flight or in the bridge -- it exists so a test can assert that
    what we send produces what we asked for, instead of the round trip being
    argued from algebra in a comment.
    """
    t_a = motor_constant * omega_a * omega_a
    t_b = motor_constant * omega_b * omega_b
    return t_a + t_b, moment_constant * (t_b - t_a)
