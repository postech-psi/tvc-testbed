"""
Uncertainty propagation over the verification scenarios. VERIFICATION support.
================================================================================
    python tvc.py validate --uncertainty [--samples N] [--seed S]

The deterministic scenarios report point values. This module reruns them under a
seeded Monte-Carlo perturbation of the inputs that carry stated uncertainty --
the lever arm L, mass and inertia, and the measured thrust/torque surface -- and
reports the SPREAD of each metric. It converts "settling time = 1.03 s" into
"1.03 s, p5-p95 = 0.97-1.12 over the stated input uncertainty", which is the
difference between Results Uncertainty level 0 and level 2 in docs/7-CREDIBILITY.

This is analysis, not flight code: it lives in verify/, imports numpy freely, and
never runs inside a control step. It is OFF by default; the deterministic path
that feeds the frozen baselines is untouched.

WHAT IS PERTURBED, AND ON WHAT BASIS
    L        uniform over [rotor_plane, pivot] -- the lever_arm_mode decision is
             DEFERRED and worth 21% of lateral authority (docs/5-PARAMETERS.md).
    mass     Gaussian, rel sigma -- CAD design intent, never weighed.
    inertia  Gaussian per principal axis (products scaled with their axes) --
             design intent; the roadmap flags +/-20-30% as the robustness range.
    surface  additive Gaussian bias of the fit RMSE (0.243 N thrust,
             0.0032 N*m torque) applied to the constant term of each polynomial.

The mass/inertia sigmas are ESTIMATES, not measurements, and are stated here so a
reader knows the spread rests on them; the L range and the surface sigmas are
sourced from the measured YAML.
"""
from dataclasses import dataclass, replace

import numpy as np

from ..gnc.effectiveness import ThrustTorqueSurface

# Estimated relative 1-sigma on the CAD mass properties. Documented assumptions,
# not measurements -- the vehicle has never been weighed or balanced.
#
# INERTIA IS DELIBERATELY WIDE. The inertia tensor is CAD design intent and the
# built vehicle has NEVER been measured -- 748 g of electronics sit at *assumed*
# positions (docs/8-ROADMAP.md ranks this #2 of what is genuinely uncertain).
# So we keep the CAD values as the mean (we still believe them as the best
# estimate) but spread widely around them: sigma 0.25 puts +/-1 sigma at 25% and
# the bulk of the mass within the +/-20-30% the roadmap flags as the real range.
# Narrow it only once the assembled vehicle's inertia has actually been measured.
_MASS_REL_SIGMA = 0.05
_INERTIA_REL_SIGMA = 0.25

# Clamp each inertia scale factor to stay physical (no zero/negative inertia) at
# the far tail of a wide Gaussian, while leaving the +/-2 sigma body untouched.
_INERTIA_FACTOR_MIN = 0.4
_INERTIA_FACTOR_MAX = 1.6


@dataclass
class UncertaintySpec:
    """The perturbation distributions, in one auditable place.

    L is drawn uniformly on [L_min, L_max]; mass and each principal inertia are
    scaled by 1 + sigma * N(0,1); the surface polynomials get an additive bias of
    sigma * N(0,1) on their constant term.
    """
    L_min: float
    L_max: float
    mass_rel_sigma: float
    inertia_rel_sigma: float
    thrust_surface_sigma_n: float
    torque_surface_sigma_nm: float

    @classmethod
    def zero(cls, L_nominal):
        """No perturbation: perturb_params returns an equivalent vehicle."""
        return cls(L_min=L_nominal, L_max=L_nominal, mass_rel_sigma=0.0,
                   inertia_rel_sigma=0.0, thrust_surface_sigma_n=0.0,
                   torque_surface_sigma_nm=0.0)

    @classmethod
    def default(cls, vehicle=None):
        """Build the spec from the YAML: measured surface RMSEs and the
        pivot/rotor_plane lever-arm span, plus the estimated mass/inertia sigmas."""
        from ..config import load
        v = vehicle or load()
        L_pivot = v.cg[2]
        L_rotor = v.cg[2] - v.rotor_z
        fit = (v.raw.get("thrust_torque_surface") or {}).get("fit") or {}
        return cls(
            L_min=min(L_pivot, L_rotor),
            L_max=max(L_pivot, L_rotor),
            mass_rel_sigma=_MASS_REL_SIGMA,
            inertia_rel_sigma=_INERTIA_REL_SIGMA,
            thrust_surface_sigma_n=(fit.get("thrust") or {}).get("rmse_n", 0.243),
            torque_surface_sigma_nm=(fit.get("torque") or {}).get("rmse_nm", 0.0032),
        )


def perturb_params(vp, spec, rng):
    """Return a VehicleParams copy with L, mass, inertia and the surface perturbed.

    Draw order is fixed so a test can reproduce a single draw: mass, then the
    three principal-inertia factors, then L, then the two surface biases. Every
    draw is scaled by its sigma, so a zero spec advances the rng but leaves every
    value unchanged -- which is what makes the toggle-off path bit-identical.
    """
    m_z = rng.standard_normal()
    ix_z = rng.standard_normal()
    iy_z = rng.standard_normal()
    iz_z = rng.standard_normal()
    L = rng.uniform(spec.L_min, spec.L_max)
    dT = spec.thrust_surface_sigma_n * rng.standard_normal()
    dQ = spec.torque_surface_sigma_nm * rng.standard_normal()

    m = vp.m * (1.0 + spec.mass_rel_sigma * m_z)

    def _ifac(z):
        # Wide, but clamped off the nonphysical tail (see _INERTIA_FACTOR_*).
        f = 1.0 + spec.inertia_rel_sigma * z
        return min(max(f, _INERTIA_FACTOR_MIN), _INERTIA_FACTOR_MAX)

    fx, fy, fz = _ifac(ix_z), _ifac(iy_z), _ifac(iz_z)
    # Products of inertia scale with the geometric mean of their two axes' factors
    # so the tensor stays a plausible tensor rather than drifting off on its own.
    fxy = (fx * fy) ** 0.5
    fxz = (fx * fz) ** 0.5
    fyz = (fy * fz) ** 0.5

    surface = vp.surface
    if surface is not None and (spec.thrust_surface_sigma_n
                                or spec.torque_surface_sigma_nm):
        cT = list(surface.c_T)
        cQ = list(surface.c_Q)
        cT[0] += dT
        cQ[0] += dQ
        surface = ThrustTorqueSurface(
            cT, cQ, pwm_offset=surface.offset, pwm_scale=surface.scale,
            pwm_min=surface.pwm_min, pwm_max=surface.pwm_max)

    return replace(vp, m=m, L=L,
                   Ix=vp.Ix * fx, Iy=vp.Iy * fy, Iz=vp.Iz * fz,
                   Ixy=vp.Ixy * fxy, Ixz=vp.Ixz * fxz, Iyz=vp.Iyz * fyz,
                   surface=surface)


# Metrics worth summarizing per scenario. Others exist but these are the ones a
# reader reaches for; the runner aggregates whatever the scenario produced.
def _numeric_metrics(metrics):
    return {k: v for k, v in metrics.items()
            if isinstance(v, (int, float)) and not isinstance(v, bool)}


def run_uncertainty(scenario_fn, vp, gains, spec, n_samples, seed,
                    sim_overrides=None):
    """Rerun one scenario under n_samples perturbed vehicles; return metric spread.

    Returns {metric_name: {mean, std, p5, p95, min, max, n}}. Deterministic in
    (scenario, spec, n_samples, seed): the same seed yields the same spread.
    sim_overrides is passed through to the scenario, so uncertainty can be
    combined with the battery-sag / aero fidelity toggles.
    """
    rng = np.random.default_rng(seed)
    samples = {}
    for _ in range(n_samples):
        pvp = perturb_params(vp, spec, rng)
        metrics = _numeric_metrics(
            scenario_fn(pvp, gains, False, sim_overrides=sim_overrides)[2]["metrics"])
        for k, val in metrics.items():
            samples.setdefault(k, []).append(val)

    spread = {}
    for k, vals in samples.items():
        a = np.asarray(vals, dtype=float)
        spread[k] = {
            "mean": float(np.mean(a)),
            "std": float(np.std(a)),
            "p5": float(np.percentile(a, 5)),
            "p95": float(np.percentile(a, 95)),
            "min": float(np.min(a)),
            "max": float(np.max(a)),
            "n": int(a.size),
        }
    return spread


def format_spread(name, s):
    """One line: 'metric   mean +/- std   [p5, p95]'."""
    return ("    %-22s %+.4f +/- %.4f   [p5 %+.4f, p95 %+.4f]"
            % (name, s["mean"], s["std"], s["p5"], s["p95"]))
