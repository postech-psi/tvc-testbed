"""
Vehicle and gain parameters -- the plain data the controller reads.
================================================================================
Moved verbatim from physics.py during the flight-code/plant split. The math is
untouched; only its address changed.

NOTE ON PORTING: the YAML search-and-load below runs at IMPORT time, not in the
control path, so it does not violate the no-runtime-file-I/O rule -- but it does
mean this module cannot be lifted into firmware as-is. The purity conversion
replaces it with an explicit factory that the harness calls once at startup,
which is how PX4's parameter system works and how the C++ port will read.
"""

import os
import sys

import numpy as np
from dataclasses import dataclass, field

# =============================================================================
# 0. Single-source-of-truth defaults
# =============================================================================
# The authoritative vehicle numbers live in sim/vehicle_params.yaml (see its
# header). Both this module and the Gazebo controller read them, so mass / CG /
# inertia / lever arm cannot drift apart. When that file is not reachable -- an
# installed ROS2 context without the sim/ tree alongside -- we fall back to the
# literals below so physics.py stays importable and self-contained.

def _load_vehicle_defaults():
    """Search upward from this file for sim/vehicle_params.py and load it.
    Returns a dict of defaults, or None if the sim tree is not present."""
    here = os.path.dirname(os.path.abspath(__file__))
    d = here
    for _ in range(6):
        sim_dir = os.path.join(d, "sim")
        if os.path.isfile(os.path.join(sim_dir, "vehicle_params.py")):
            if sim_dir not in sys.path:
                sys.path.insert(0, sim_dir)
            try:
                import vehicle_params as _vp
                v = _vp.load()
            except Exception:
                return None
            try:
                import actuator_maps as _am
                surface = _am.ThrustTorqueSurface(v.raw.get("thrust_torque_surface"))
                if not surface.ok:
                    surface = None
                axes = _am.GimbalAxisMap.all_from_params() or None
            except Exception:
                surface, axes = None, None
            return {
                "m": v.mass, "Ix": v.Ix, "Iy": v.Iy, "Iz": v.Iz,
                "L": v.L,
                # dx/dy stay 0 by design: per this module's convention they are
                # OPT-IN CM-misalignment disturbance terms, not the vehicle's
                # nominal state. The real <2 mm lateral CG offset lives in the
                # SDF (link pose + products of inertia); forcing it here as a
                # constant bias torque only muddies the attitude smoke test.
                "dx": 0.0, "dy": 0.0,
                "T_max": v.thrust_at_max_n,
                "gimbal_max_deg": v.gimbal_max_deg,
                "gimbal_rate_max_deg": v.gimbal_rate_max_deg,
                "gimbal_deadtime_s": v.gimbal_deadtime_s,
                "k_moment": v.moment_constant,
                "tau_p_max": v.tau_p_max_nm,
                "surface": surface,
                "gimbal_axes": axes,
                "g": v.g,
            }
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


# Literal fallbacks used only when sim/vehicle_params.yaml cannot be found.
_D = _load_vehicle_defaults() or {
    "m": 1.328, "Ix": 0.022616, "Iy": 0.022581, "Iz": 0.001957,
    "L": 0.2111, "dx": 0.0, "dy": 0.0,
    "T_max": 20.0, "gimbal_max_deg": 7.0, "gimbal_rate_max_deg": 180.0,
    "gimbal_deadtime_s": 0.030, "k_moment": 0.016, "tau_p_max": None,
    "surface": None, "gimbal_axes": None,
    "g": 9.81,
}
_D.setdefault("surface", None)
_D.setdefault("gimbal_axes", None)


@dataclass
class VehicleParams:
    """Physical vehicle parameters. Angles are stored in DEGREES for the GUI's
    convenience and converted to radians via properties where dynamics need them.

    Defaults come from sim/vehicle_params.yaml (the single source of truth), so
    editing that file re-parameterizes this sim without touching code here."""

    m: float = _D["m"]              # kg, total mass
    Ix: float = _D["Ix"]            # kg*m^2, roll inertia (body x)
    Iy: float = _D["Iy"]            # kg*m^2, pitch inertia (body y)
    Iz: float = _D["Iz"]            # kg*m^2, yaw/spin inertia (body z)

    L: float = _D["L"]              # m, AXIAL lever arm: gimbal pivot -> CM
    dx: float = _D["dx"]            # m, lateral CM misalignment (disturbance)
    dy: float = _D["dy"]            # m, lateral CM misalignment (disturbance)

    T_max: float = _D["T_max"]      # N, max thrust (combined coax unit)
    T_min: float = 5.0              # N, min thrust (idle; must stay > 0 for allocation)

    gimbal_max_deg: float = _D["gimbal_max_deg"]        # deg, mechanical gimbal limit (each axis)
    gimbal_rate_max_deg: float = _D["gimbal_rate_max_deg"]  # deg/s, servo slew rate
    gimbal_deadtime_s: float = _D["gimbal_deadtime_s"]  # s, servo transport delay

    # Axial (thrust-axis) reaction torque authority. k_moment is the coax
    # drag-torque / thrust ratio, so tau_P = k_moment * (T2 - T1) for a thrust
    # split of (T2 - T1) newtons. tau_p_max is an optional MEASURED cap; when
    # None the allocator uses only the split headroom the thrust command leaves.
    k_moment: float = _D["k_moment"]        # m, drag-torque / thrust
    tau_p_max: float = _D["tau_p_max"]      # N*m or None

    # Bench-measured (PWM A, PWM B) -> (thrust, tau_P) surface, or None. When
    # present it REPLACES the k_moment model everywhere authority is computed;
    # k_moment survives only for the Gazebo plugin, which cannot take a surface.
    surface: object = _D["surface"]
    # Per-axis measured gimbal maps {'inner': .., 'outer': ..}, or None.
    # default_factory because a dict default is mutable and dataclasses reject
    # it -- the factory hands out the same shared, read-only spec object.
    gimbal_axes: object = field(default_factory=lambda: _D["gimbal_axes"])

    g: float = _D["g"]              # m/s^2

    @property
    def gimbal_max(self):
        return np.deg2rad(self.gimbal_max_deg)

    @property
    def gimbal_rate_max(self):
        return np.deg2rad(self.gimbal_rate_max_deg)

    # --- per-axis travel ------------------------------------------------------
    # delta1 is the pitch-plane deflection, carried by the INNER ring; delta2 is
    # the roll-plane one, carried by the OUTER ring (base_link -> roll joint ->
    # outer ring -> pitch joint -> inner ring -> rotors). Neither axis is
    # symmetric about its own neutral, and they differ from each other, so the
    # limits are vectors rather than one scalar. Falls back to the symmetric
    # +/-gimbal_max_deg when the measured block is absent.

    def _axis(self, name):
        return (self.gimbal_axes or {}).get(name)

    @property
    def delta_min(self):
        inner, outer = self._axis("inner"), self._axis("outer")
        return np.deg2rad([inner.min_deg if inner else -self.gimbal_max_deg,
                           outer.min_deg if outer else -self.gimbal_max_deg])

    @property
    def delta_max(self):
        inner, outer = self._axis("inner"), self._axis("outer")
        return np.deg2rad([inner.max_deg if inner else self.gimbal_max_deg,
                           outer.max_deg if outer else self.gimbal_max_deg])

    @property
    def delta_rate_max(self):
        inner, outer = self._axis("inner"), self._axis("outer")
        return np.deg2rad([inner.rate_max_deg if inner else self.gimbal_rate_max_deg,
                           outer.rate_max_deg if outer else self.gimbal_rate_max_deg])


@dataclass
class ControlGains:
    """Cascaded PID gains: outer angle loop + inner rate loop.

    LATERAL (body x, y) gains are shared between the two axes -- Ix and Iy
    differ by 0.15% on this airframe, so treating them as symmetric is exact
    to within the mass-budget uncertainty.

    AXIAL (body z) gains are SEPARATE and much smaller, and this is not a
    tuning preference -- it is forced by the airframe. Iz = 0.00196 kg*m^2 is
    11.6x smaller than Ix, so the same torque produces 11.6x the angular
    acceleration about z. Sharing one gain set makes the axial channel
    violently underdamped (or the lateral channel uselessly slow). Also the
    two channels have different actuators and therefore different lags: the
    lateral axes go through a 30 ms servo deadtime, the axial channel through
    the motor time constant.
    """

    kp_angle: float = 4.0      # outer loop: angle error -> rate setpoint
    kp_rate: float = 0.02      # inner loop: rate error -> torque
    ki_rate: float = 0.002
    kd_rate: float = 0.004
    i_limit: float = 0.5       # integrator clamp (anti-windup)

    # --- axial channel (body z / tau_P), scaled by Iz/Ix ~= 1/11.6 ---
    kp_angle_axial: float = 4.0
    kp_rate_axial: float = 0.0018
    ki_rate_axial: float = 0.00018
    kd_rate_axial: float = 0.00035
    i_limit_axial: float = 0.05

    # --- altitude cascade: outer P (m -> m/s), inner PID (m/s -> m/s^2) ---
    kp_alt: float = 1.5        # m/s of climb demand per m of altitude error
    kp_vz: float = 4.0         # m/s^2 per m/s
    ki_vz: float = 1.0
    kd_vz: float = 0.2
    i_limit_vz: float = 4.0    # m/s^2 of integral authority
    vz_max: float = 2.0        # m/s, climb/descent rate limit
