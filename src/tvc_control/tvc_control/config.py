"""
config.py -- the ONLY place vehicle numbers enter the program.
================================================================================
This module reads vehicle_params.yaml and builds the plain structs the flight
code consumes. It is deliberately NOT in `gnc/`: flight code does no file I/O,
and a parameter block is injected once at startup rather than looked up. That is
how PX4's parameter system works, and keeping the same shape is what makes the
eventual C++ port a port rather than a redesign.

There is no fallback. Previous versions of this project carried literal default
constants for the case where the YAML could not be found, and those defaults
went stale -- a superseded 20.0 N max thrust survived in three separate files
after the bench measured 17.79 N, and a 180 deg/s slew rate outlived its
measurement at 235/403. A number that can silently disagree with the source of
truth WILL, so a missing or unreadable YAML is now an error.
Every simulator imports this instead of hard-coding mass / inertia / lever-arm /
thrust constants, so there is exactly one place those numbers live. See
sim/vehicle_params.yaml for the data and the provenance of each field.

Usage:
    from vehicle_params import load
    vp = load()
    vp.mass, vp.Ix, vp.Iy, vp.Iz     # kg, kg*m^2
    vp.L                             # m, control lever arm (mode-dependent)
    vp.cg                            # (x, y, z) in METERS
    vp.gimbal_max_deg, vp.gimbal_rate_max_deg
    vp.max_rot_velocity, vp.motor_constant, vp.thrust_at_max_n, vp.moment_constant
    vp.tau_p_max_nm                  # N*m or None (see the YAML)
    vp.gimbal_deadtime_s             # s
    vp.g
    vp.raw                           # the full parsed dict, for less-common fields
"""
import os
from dataclasses import dataclass

import yaml

_DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "vehicle_params.yaml")

# The axis convention the YAML (and therefore every consumer) is written in.
# See docs/CONVENTIONS.md. Loading a file that predates or postdates the rename
# must fail loudly rather than reinterpret its numbers under the wrong names.
SUPPORTED_AXIS_CONVENTIONS = ("rocket_v2",)


@dataclass
class Vehicle:
    """Flattened, unit-normalized view of vehicle_params.yaml.

    Lengths are METERS, masses kg, inertia kg*m^2, angles as noted. The raw
    nested dict is kept on .raw for fields without a convenience accessor
    (PWM maps, rotor drag coefficients, etc.)."""
    mass: float
    cg: tuple            # (x, y, z) meters
    Ix: float
    Iy: float
    Iz: float
    Ixy: float
    Ixz: float
    Iyz: float
    rotor_z: float       # meters
    lever_arm_mode: str
    L: float             # meters, derived from mode
    gimbal_max_deg: float
    gimbal_rate_max_deg: float
    gimbal_deadtime_s: float
    max_rot_velocity: float
    motor_constant: float
    moment_constant: float
    thrust_at_max_n: float
    tau_p_max_nm: float  # N.m, optional hard cap on roll torque, or None
    surface: dict        # measured (PWM A, PWM B) -> (thrust, torque) cubic fit
    gimbal_axes: dict    # per-axis measured gimbal data ('inner' / 'outer')
    motor_dynamics: dict # measured motor lag / sag / derating
    g: float
    raw: dict

    @property
    def weight_n(self):
        return self.mass * self.g


def load(path=None):
    """Parse vehicle_params.yaml into a Vehicle. Raises on a missing/invalid
    lever_arm_mode rather than silently guessing a lever arm."""
    path = path or _DEFAULT_PATH
    if not os.path.isfile(path):
        raise FileNotFoundError(
            "vehicle_params.yaml not found at %s. There is no fallback: a "
            "default that can disagree with the measured source of truth is "
            "exactly the bug this project keeps removing." % path)
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)

    conv = d.get("axis_convention", "rocket_v2")
    if conv not in SUPPORTED_AXIS_CONVENTIONS:
        raise ValueError(
            "unknown axis_convention %r in %s -- see docs/CONVENTIONS.md"
            % (conv, path))

    mp = d["mass_properties"]
    cg_m = tuple(v / 1000.0 for v in mp["cg_mm"])
    I = mp["inertia_kg_m2"]

    geom = d["geometry"]
    rotor_z = geom["rotor_z_mm"] / 1000.0
    mode = geom["lever_arm_mode"]
    if mode == "pivot":
        L = cg_m[2]
    elif mode == "rotor_plane":
        L = cg_m[2] - rotor_z
    else:
        raise ValueError(
            "geometry.lever_arm_mode must be 'pivot' or 'rotor_plane', got %r"
            % mode)

    gim = d["gimbal"]
    rot = d["rotors"]

    return Vehicle(
        mass=mp["mass_kg"],
        cg=cg_m,
        Ix=I["ixx"], Iy=I["iyy"], Iz=I["izz"],
        Ixy=I["ixy"], Ixz=I["ixz"], Iyz=I["iyz"],
        rotor_z=rotor_z,
        lever_arm_mode=mode,
        L=L,
        gimbal_max_deg=gim["max_deg"],
        gimbal_rate_max_deg=gim["rate_max_deg"],
        gimbal_deadtime_s=gim.get("deadtime_s", 0.0),
        max_rot_velocity=rot["max_rot_velocity"],
        motor_constant=rot["motor_constant"],
        moment_constant=rot["moment_constant"],
        thrust_at_max_n=rot["thrust_at_max_n"],
        tau_p_max_nm=rot.get("tau_p_max_nm"),
        surface=d.get("thrust_torque_surface") or {},
        gimbal_axes=gim.get("axes") or {},
        motor_dynamics=d.get("motor_dynamics") or {},
        g=d["env"]["gravity"],
        raw=d,
    )


def load_vehicle_params(path=None):
    """Build the flight code's VehicleParams from the YAML. Call once, at startup.

    This is the ONLY supported way to obtain a VehicleParams. The dataclass has
    no defaults for physical quantities on purpose -- constructing one without
    numbers is a TypeError rather than a silently wrong vehicle.
    """
    from .gnc.params import VehicleParams
    from .gnc.effectiveness import ThrustTorqueSurface, GimbalAxisMap

    v = load(path)

    surface = None
    spec = v.surface or {}
    if spec.get("type") == "cubic_ab":
        surface = ThrustTorqueSurface(
            spec["thrust_n"], spec["torque_nm"],
            pwm_offset=spec.get("pwm_offset", 1500.0),
            pwm_scale=spec.get("pwm_scale", 500.0),
            pwm_min=spec.get("pwm_min", 1000.0),
            pwm_max=spec.get("pwm_max", 2000.0),
        )

    axes = {k: GimbalAxisMap(
        a["gain_deg_per_us"], a["neutral_pwm"], a["min_deg"], a["max_deg"],
        a.get("rate_max_deg", 180.0), a.get("bandwidth_hz", 0.0),
    ) for k, a in (v.gimbal_axes or {}).items()} or None

    return VehicleParams(
        m=v.mass, Ix=v.Ix, Iy=v.Iy, Iz=v.Iz,
        Ixy=v.Ixy, Ixz=v.Ixz, Iyz=v.Iyz,
        L=v.L,
        # dx/dy stay 0 by design: they are OPT-IN CM-misalignment disturbance
        # terms, not the vehicle's nominal state. The real <2 mm lateral CG
        # offset lives in the SDF (link pose + products of inertia); forcing it
        # here as a constant bias torque only muddies the attitude smoke test.
        dx=0.0, dy=0.0,
        T_max=v.thrust_at_max_n,
        gimbal_max_deg=v.gimbal_max_deg,
        gimbal_rate_max_deg=v.gimbal_rate_max_deg,
        gimbal_deadtime_s=v.gimbal_deadtime_s,
        k_moment=v.moment_constant,
        tau_p_max=v.tau_p_max_nm,
        surface=surface,
        gimbal_axes=axes,
        g=v.g,
    )


_GAINS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "control_gains.yaml")


def load_gains(profile=None, path=None):
    """Build ControlGains from a named profile in control_gains.yaml.

    profile=None takes the file's own default_profile, so switching the whole
    project between gain sets is a one-line edit in one file rather than a
    search for constructor call sites.
    """
    from .gnc.params import ControlGains

    path = path or _GAINS_PATH
    if not os.path.isfile(path):
        raise FileNotFoundError("control_gains.yaml not found at %s" % path)
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)

    name = profile or d.get("default_profile")
    profiles = d.get("profiles") or {}
    if name not in profiles:
        raise ValueError("unknown gain profile %r; have %s"
                         % (name, sorted(profiles)))
    p = profiles[name]
    att, ax = p["attitude"], p["roll"]
    alt, pos = p["altitude"], p["position"]
    return ControlGains(
        kp_angle=att["kp_angle"], kp_rate=att["kp_rate"],
        ki_rate=att["ki_rate"], kd_rate=att["kd_rate"], i_limit=att["i_limit"],
        kp_angle_roll=ax["kp_angle"], kp_rate_roll=ax["kp_rate"],
        ki_rate_roll=ax["ki_rate"], kd_rate_roll=ax["kd_rate"],
        i_limit_roll=ax["i_limit"],
        kp_alt=alt["kp_alt"], kp_vz=alt["kp_vz"], ki_vz=alt["ki_vz"],
        kd_vz=alt["kd_vz"], i_limit_vz=alt["i_limit_vz"], vz_max=alt["vz_max"],
        kp_pos=pos["kp_pos"], kd_pos=pos["kd_pos"],
        max_tilt_deg=pos["max_tilt_deg"],
    )


if __name__ == "__main__":
    vp = load()
    print("Loaded sim/vehicle_params.yaml:")
    print("  mass         %.4f kg" % vp.mass)
    print("  CG           (%.1f, %.1f, %.1f) mm"
          % tuple(c * 1000 for c in vp.cg))
    print("  inertia      Ix=%.6f Iy=%.6f Iz=%.6f" % (vp.Ix, vp.Iy, vp.Iz))
    print("  lever arm L  %.4f m   (mode=%s, rotor_z=%.1f mm)"
          % (vp.L, vp.lever_arm_mode, vp.rotor_z * 1000))
    print("  gimbal       +/-%.1f deg, %.0f deg/s, deadtime %.0f ms"
          % (vp.gimbal_max_deg, vp.gimbal_rate_max_deg,
             vp.gimbal_deadtime_s * 1000))
    print("  tau_P cap    %s"
          % ("%.4f N.m (measured)" % vp.tau_p_max_nm
             if vp.tau_p_max_nm else "analytic headroom (none measured)"))
    print("  thrust max   %.2f N   (T/W = %.2f)"
          % (vp.thrust_at_max_n, vp.thrust_at_max_n / vp.weight_n))
    print("  surface      %s" % (vp.surface.get("type") or "NONE (analytic fallback)"))
    for name in ("inner", "outer"):
        ax = vp.gimbal_axes.get(name)
        if ax:
            print("  gimbal %-5s %+.2f .. %+.2f deg, %.4f deg/us @ %.1f us, "
                  "%.0f deg/s, BW %.0f Hz"
                  % (name, ax["min_deg"], ax["max_deg"], ax["gain_deg_per_us"],
                     ax["neutral_pwm"], ax["rate_max_deg"], ax["bandwidth_hz"]))
