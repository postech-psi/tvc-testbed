"""
Tests for the parametric aero model (plant/aero.py) and its integration into the
rigid-body dynamics. Toggle-off must leave the dynamics bit-identical.
"""
import numpy as np

from tvc_control.plant.aero import drag_force_inertial, ground_effect_factor


def test_drag_zero_at_rest():
    assert np.array_equal(drag_force_inertial([0, 0, 0], 0.8, 0.03, 1.225),
                          np.zeros(3))


def test_drag_opposes_velocity_and_scales_quadratically():
    f1 = drag_force_inertial([1.0, 0, 0], 0.8, 0.03, 1.225)
    f2 = drag_force_inertial([2.0, 0, 0], 0.8, 0.03, 1.225)
    assert f1[0] < 0                              # opposes +x motion
    # |F| ~ v^2, so doubling speed quadruples the force.
    assert abs(np.linalg.norm(f2) - 4 * np.linalg.norm(f1)) < 1e-9


def test_drag_disabled_by_zero_coeffs():
    assert np.array_equal(drag_force_inertial([3, 0, 0], 0.0, 0.03, 1.225),
                          np.zeros(3))


def test_ground_effect_tends_to_one_at_altitude():
    assert abs(ground_effect_factor(100.0, 0.12) - 1.0) < 1e-3


def test_ground_effect_exceeds_one_near_ground_and_is_bounded():
    near = ground_effect_factor(0.05, 0.12)
    assert near > 1.0
    # Clamped at z = R/2, so the factor never exceeds 4/3.
    assert near <= 4.0 / 3.0 + 1e-9
    assert ground_effect_factor(0.0, 0.12) <= 4.0 / 3.0 + 1e-9   # finite at z=0


def test_ground_effect_monotonic_in_altitude():
    lo = ground_effect_factor(0.2, 0.12)
    hi = ground_effect_factor(1.0, 0.12)
    assert lo > hi > 1.0 - 1e-9


# --- integration into the rigid body -----------------------------------------
def _state(v=(0.0, 0.0, 0.0), z=2.0):
    from tvc_control.gnc.mathx import euler_to_quat
    q = euler_to_quat(0.0, 0.0, 0.0)
    return np.concatenate([[0, 0, z], list(v), q, [0, 0, 0]])


def test_dynamics_aero_off_is_bit_identical():
    from dataclasses import replace
    from tvc_control.config import load_vehicle_params
    from tvc_control.plant.rigidbody import dynamics
    vp = load_vehicle_params()                    # aero_enabled=False in YAML
    vp_on_fields = replace(vp, aero_enabled=True, cd=0.0, ref_area=0.0,
                           rotor_radius=0.0)       # enabled but zeroed -> no effect
    x = _state(v=(1.0, -0.5, 0.3), z=1.5)
    T, delta = 13.0, np.array([0.02, -0.01])
    d_off = dynamics(0.0, x, T, delta, vp)
    d_zeroed = dynamics(0.0, x, T, delta, vp_on_fields)
    assert np.allclose(d_off, d_zeroed, atol=0, rtol=0)


def test_drag_removes_translational_kinetic_energy():
    """With drag on, no gravity and no control, translational KE must not grow."""
    from dataclasses import replace
    from scipy.integrate import solve_ivp
    from tvc_control.config import load_vehicle_params
    from tvc_control.plant.rigidbody import dynamics
    vp = load_vehicle_params()
    vp = replace(vp, aero_enabled=True, cd=0.8, ref_area=0.03, rho=1.225,
                 rotor_radius=0.0, g=0.0)          # drag only, no gravity/GE
    x0 = _state(v=(3.0, 2.0, -1.0), z=50.0)
    sol = solve_ivp(dynamics, [0, 2.0], x0, args=(0.0, np.zeros(2), vp),
                    method="RK45", max_step=0.01)
    ke0 = 0.5 * vp.m * np.sum(x0[3:6] ** 2)
    ke1 = 0.5 * vp.m * np.sum(sol.y[3:6, -1] ** 2)
    assert ke1 <= ke0 + 1e-9
    assert ke1 < ke0                               # drag actually removed energy
