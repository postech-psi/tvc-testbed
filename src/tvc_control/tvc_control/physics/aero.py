"""Optional estimated drag and ground effect; these parameters are not measured."""
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
