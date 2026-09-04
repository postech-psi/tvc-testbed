"""
vehicle_params.py -- loader for the single-source-of-truth vehicle_params.yaml.
================================================================================
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
    tau_p_max_nm: float  # N.m, optional hard cap on axial torque, or None
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
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)

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
