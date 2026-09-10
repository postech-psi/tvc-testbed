"""
Aerodynamics -- simulation only, and ESTIMATED rather than measured.
================================================================================
There is NO aero bench data for this vehicle: no wind tunnel, no drag run. So
unlike the thrust surface or the gimbal maps, nothing here is metrology. These
are physics-based parametric estimates whose job is to let a scenario ask "does a
plausible amount of drag or ground effect change a conclusion?" -- an input to
robustness, flagged provisional wherever its output is used.

Two effects, the two most likely to matter for a VTVL hover-and-land demonstrator:

    quadratic drag   F = -0.5 * rho * Cd * A * |v| * v, opposing inertial
                     velocity. Zero at rest, grows with v^2.
    ground effect    a thrust multiplier that rises near the ground, from the
                     standard rotor image model T/T_inf = 1/(1 - (R/4z)^2). This
                     is the one docs/8-ROADMAP flags as "most likely to bite"
                     during landing.

Rotational aero damping is deliberately NOT modelled here (documented as
deferred): it needs its own coefficients, and none are even estimable yet.
Everything is OFF by default via params.aero_enabled.
"""
import numpy as np


def drag_force_inertial(v_inertial, cd, ref_area, rho):
    """Quadratic drag opposing the inertial velocity, in newtons (inertial frame).

    Returns a zero vector at rest or when any coefficient is non-positive, so an
    unconfigured aero block contributes nothing.
    """
    v = np.asarray(v_inertial, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed == 0.0 or cd <= 0.0 or ref_area <= 0.0 or rho <= 0.0:
        return np.zeros(3)
    return -0.5 * rho * cd * ref_area * speed * v


def ground_effect_factor(z, rotor_radius):
    """Thrust multiplier from rotor-in-ground-effect, >= 1, bounded.

    T/T_inf = 1 / (1 - (R / 4z)^2). It rises as the rotor nears the ground and
    tends to 1 at altitude. The altitude is clamped at R/2 so the factor stays
    finite and bounded (its cap there is 4/3), rather than blowing up at z -> 0
    where the image model stops being physical anyway.
    """
    if rotor_radius <= 0.0:
        return 1.0
    z_eff = max(float(z), rotor_radius / 2.0)
    ratio = rotor_radius / (4.0 * z_eff)
    return max(1.0, 1.0 / (1.0 - ratio * ratio))
