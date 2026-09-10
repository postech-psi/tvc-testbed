"""
Tests for uncertainty propagation (verify/uncertainty.py).

The load-bearing test is `test_zero_spec_reproduces_metrics`: with the
uncertainty switched off, every scenario metric must be bit-identical to the
deterministic run, which is what lets --uncertainty exist without moving the
frozen goldens.
"""
import numpy as np

from tvc_control.config import load_gains, load_vehicle_params
from tvc_control.verify import uncertainty as U
from tvc_control.verify.scenarios import scenario_lateral


def _vp_gains():
    return load_vehicle_params(), load_gains()


def test_zero_spec_returns_identical_params_and_metrics():
    vp, gains = _vp_gains()
    spec = U.UncertaintySpec.zero(vp.L)
    rng = np.random.default_rng(0)
    pvp = U.perturb_params(vp, spec, rng)
    # Fields untouched.
    assert pvp.m == vp.m
    assert pvp.L == vp.L
    assert (pvp.Ix, pvp.Iy, pvp.Iz) == (vp.Ix, vp.Iy, vp.Iz)
    # And the closed-loop metrics are therefore identical.
    m0 = scenario_lateral(vp, gains, False)[2]["metrics"]
    m1 = scenario_lateral(pvp, gains, False)[2]["metrics"]
    assert m0 == m1


def test_mass_draw_matches_zscore():
    vp, gains = _vp_gains()
    spec = U.UncertaintySpec(L_min=vp.L, L_max=vp.L, mass_rel_sigma=0.05,
                             inertia_rel_sigma=0.0, thrust_surface_sigma_n=0.0,
                             torque_surface_sigma_nm=0.0)
    # perturb_params draws the mass z-score FIRST from its rng.
    z = np.random.default_rng(42).standard_normal()
    pvp = U.perturb_params(vp, spec, np.random.default_rng(42))
    assert abs(pvp.m - vp.m * (1.0 + 0.05 * z)) < 1e-12


def test_surface_perturbation_is_additive_thrust_offset():
    vp, gains = _vp_gains()
    assert vp.surface is not None
    spec = U.UncertaintySpec(L_min=vp.L, L_max=vp.L, mass_rel_sigma=0.0,
                             inertia_rel_sigma=0.0, thrust_surface_sigma_n=0.243,
                             torque_surface_sigma_nm=0.0)
    pvp = U.perturb_params(vp, spec, np.random.default_rng(7))
    d1 = pvp.surface.forward_norm(0.0, 0.0)[0] - vp.surface.forward_norm(0.0, 0.0)[0]
    d2 = pvp.surface.forward_norm(0.5, 0.3)[0] - vp.surface.forward_norm(0.5, 0.3)[0]
    assert abs(d1 - d2) < 1e-12   # a pure offset, independent of (a, b)
    assert abs(d1) > 0.0


def test_default_spec_sources_from_yaml():
    spec = U.UncertaintySpec.default()
    # L range spans the pivot <-> rotor_plane decision (~21%).
    assert spec.L_max > spec.L_min > 0.0
    assert (spec.L_max - spec.L_min) / spec.L_max > 0.15
    # Surface sigmas are the measured fit RMSEs.
    assert abs(spec.thrust_surface_sigma_n - 0.243) < 1e-6
    assert abs(spec.torque_surface_sigma_nm - 0.0032) < 1e-6


def test_run_uncertainty_zero_spec_is_point_value():
    vp, gains = _vp_gains()
    spec = U.UncertaintySpec.zero(vp.L)
    spread = U.run_uncertainty(scenario_lateral, vp, gains, spec,
                               n_samples=3, seed=0)
    det = scenario_lateral(vp, gains, False)[2]["metrics"]
    for k, s in spread.items():
        assert s["std"] < 1e-9           # no perturbation -> zero spread
        assert abs(s["mean"] - det[k]) < 1e-9


def test_run_uncertainty_is_seed_reproducible():
    vp, gains = _vp_gains()
    spec = U.UncertaintySpec.default()
    a = U.run_uncertainty(scenario_lateral, vp, gains, spec, 4, seed=1)
    b = U.run_uncertainty(scenario_lateral, vp, gains, spec, 4, seed=1)
    assert a == b


def test_cli_uncertainty_flag_runs_and_exits_clean():
    from tvc_control.verify import scenarios
    # A tiny sample count keeps the test fast; the flag must run without raising
    # and return the usual pass/fail exit code.
    rc = scenarios.main(["--uncertainty", "--samples", "2", "--seed", "0"])
    assert rc in (0, 1)


def test_run_uncertainty_spread_grows_with_sigma():
    vp, gains = _vp_gains()
    small = U.UncertaintySpec(L_min=vp.L, L_max=vp.L, mass_rel_sigma=0.02,
                              inertia_rel_sigma=0.02, thrust_surface_sigma_n=0.0,
                              torque_surface_sigma_nm=0.0)
    big = U.UncertaintySpec(L_min=vp.L, L_max=vp.L, mass_rel_sigma=0.2,
                            inertia_rel_sigma=0.2, thrust_surface_sigma_n=0.0,
                            torque_surface_sigma_nm=0.0)
    metric = "max_gimbal_inner_deg"
    s_small = U.run_uncertainty(scenario_lateral, vp, gains, small, 10, seed=3)
    s_big = U.run_uncertainty(scenario_lateral, vp, gains, big, 10, seed=3)
    assert s_big[metric]["std"] > s_small[metric]["std"]
