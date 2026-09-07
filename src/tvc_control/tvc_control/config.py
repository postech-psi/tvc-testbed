"""
config.py -- the ONLY place vehicle numbers enter the program.
================================================================================
Reads vehicle_params.yaml and control_gains.yaml and builds the plain structs the
flight code consumes.

WHY THIS IS NOT IN gnc/
    Flight code does no file I/O. A parameter block is injected once at startup,
    never looked up during a control step. That is how PX4's parameter system
    works, and keeping the same shape is what makes the eventual C++ port a port
    rather than a redesign. Keep this boundary explicit in code review.

WHY THERE IS NO FALLBACK
    Earlier versions carried literal default constants for the case where the
    YAML could not be found. Those defaults went stale: a superseded 20.0 N max
    thrust survived in three separate files after the bench measured 17.79 N,
    and a 180 deg/s gimbal slew outlived its measurement at 235/403 deg/s.
    Nothing failed -- the simulation quietly described a different vehicle.
    A number that CAN silently disagree with the source of truth eventually WILL,
    so a missing or unreadable YAML is now an error.

THE THREE ENTRY POINTS

    load()                 -> Vehicle       the YAML, flattened and unit-normalized
    load_vehicle_params()  -> VehicleParams what gnc/ consumes (SI, no defaults)
    load_gains(profile)    -> ControlGains  a named profile from control_gains.yaml

`Vehicle` keeps the full parsed dict on `.raw` for the fields without a
convenience accessor -- the rotor solver constants, the measured fit statistics,
the motor-dynamics block. Consumers that need those read `.raw`, so adding a
field to the YAML does not require touching this file.

Print everything, with provenance:

    python tvc.py params
"""
import os
from dataclasses import dataclass

import yaml

_DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "vehicle_params.yaml")

# The axis convention the YAML (and therefore every consumer) is written in.
# See docs/4-CONVENTIONS.md. Loading a file that predates or postdates the rename
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
        """Vehicle weight in newtons, mass * gravity."""
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
            "unknown axis_convention %r in %s -- see docs/4-CONVENTIONS.md"
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


# =============================================================================
# The parameter report: every number, its value, and where it came from.
# =============================================================================
# ONE generator, two consumers: `python tvc.py params` prints it, and
# tools/gen_docs.py splices it into docs/5-PARAMETERS.md between EMIT markers.
# A documented number that can drift from the YAML is a number that will, so the
# document is generated and CI fails when it is stale.
#
# The `source` column is the point of the table. "measured" and "estimated" are
# different kinds of claim, and a reader deciding how far to trust a simulation
# result needs to see which is which without opening the YAML.

# provenance tags, in decreasing order of how much they are worth
MEASURED = "measured"        # from a bench run, with a stated fit error
DERIVED = "derived"          # computed from measured or CAD inputs
CAD = "CAD"                  # from the STEP/STL model: design intent, not metrology
ESTIMATED = "estimated"      # someone's assumption; no measurement exists
CHOSEN = "chosen"            # a modelling or control decision, not a property
SOLVER = "solver"            # a constant that exists to make a numeric method work


def parameter_rows(v=None):
    """-> [(section, name, value, unit, source, note)] for the whole vehicle."""
    v = v or load()
    surf, md = v.raw.get("thrust_torque_surface") or {}, v.motor_dynamics
    fit = surf.get("fit") or {}
    R = []

    R.append(("Mass properties", "mass", "%.4f" % v.mass, "kg", CAD,
              "components.yaml + a lumped remainder to the 1328 g design total"))
    for i, ax in enumerate("xyz"):
        R.append(("Mass properties", "CG %s" % ax, "%+.1f" % (v.cg[i] * 1000),
                  "mm", DERIVED, "computed from the itemized masses"))
    for nm, val in (("Ixx", v.Ix), ("Iyy", v.Iy), ("Izz", v.Iz)):
        R.append(("Mass properties", nm, "%.6f" % val, "kg*m^2", DERIVED,
                  "mesh inertia for CAD solids, m*d^2 for point masses"))
    for nm, val in (("Ixy", v.Ixy), ("Ixz", v.Ixz), ("Iyz", v.Iyz)):
        R.append(("Mass properties", nm, "%+.6f" % val, "kg*m^2", DERIVED,
                  "product of inertia -- NOT negligible here: Iyz is %.0f%% "
                  "of Izz" % (100 * abs(v.Iyz) / v.Iz) if nm == "Iyz"
                  else "product of inertia, carried in full"))

    R.append(("Geometry", "lever arm L", "%.4f" % v.L, "m", ESTIMATED,
              "mode=%s; UNMEASURED and it sets all lateral authority"
              % v.lever_arm_mode))
    R.append(("Geometry", "rotor plane z", "%.1f" % (v.rotor_z * 1000), "mm", CAD,
              "where the multicopter plugin applies thrust"))

    R.append(("Gimbal", "travel (nominal)", "+/-%.1f" % v.gimbal_max_deg, "deg",
              MEASURED, "symmetric summary; the SDF joint stops use it"))
    for name in ("inner", "outer"):
        a = (v.gimbal_axes or {}).get(name)
        if not a:
            continue
        plane = "yaw plane (body y)" if name == "inner" else "pitch plane (body x)"
        R.append(("Gimbal", "%s travel" % name,
                  "%+.2f .. %+.2f" % (a["min_deg"], a["max_deg"]), "deg", MEASURED,
                  "%s; asymmetric about its own neutral" % plane))
        R.append(("Gimbal", "%s slew" % name, "%.0f" % a["rate_max_deg"],
                  "deg/s", MEASURED, "peak measured rate"))
        R.append(("Gimbal", "%s bandwidth" % name, "%.0f" % a["bandwidth_hz"],
                  "Hz", MEASURED, "-3 dB; resonance +%.0f dB is NOT modelled"
                  % a.get("resonance_db", 0.0)))
        f = a.get("fit") or {}
        R.append(("Gimbal", "%s angle fit" % name,
                  "RMSE %.3f" % f.get("rmse_deg", 0.0), "deg", MEASURED,
                  "R2 %.4f, hysteresis up to %.2f deg (unmodelled)"
                  % (f.get("r2", 0.0), f.get("hysteresis_max_deg", 0.0))))
    R.append(("Gimbal", "transport deadtime", "%.0f" % (v.gimbal_deadtime_s * 1000),
              "ms", MEASURED, "pure delay; costs phase at every frequency"))

    R.append(("Rotors", "max thrust", "%.2f" % v.thrust_at_max_n, "N", MEASURED,
              "f_T(1,1) of the surface; T/W = %.2f" % (v.thrust_at_max_n / v.weight_n)))
    R.append(("Rotors", "motor_constant", "%.3g" % v.motor_constant,
              "N/(rad/s)^2", DERIVED, "from max thrust; used by the gz plugin"))
    R.append(("Rotors", "moment_constant", "%.3g" % v.moment_constant, "m", SOLVER,
              "NOT a drag ratio -- scaling for the Gazebo command-side inversion"))
    R.append(("Rotors", "max_rot_velocity", "%.0f" % v.max_rot_velocity, "rad/s",
              SOLVER, "solver headroom, not a physical RPM limit"))
    R.append(("Rotors", "rotor drag / rolling", "0.0", "-", CHOSEN,
              "zeroed for plant parity: they scale with a fictitious omega"))

    if surf:
        R.append(("Thrust/torque surface", "form", surf.get("type", "-"), "-",
                  MEASURED, "cubic in a=(A-1500)/500, b=(B-1500)/500"))
        t, q = fit.get("thrust") or {}, fit.get("torque") or {}
        R.append(("Thrust/torque surface", "thrust fit",
                  "RMSE %.3f" % t.get("rmse_n", 0.0), "N", MEASURED,
                  "R2 %.4f over 119 of 121 swept points" % t.get("r2", 0.0)))
        R.append(("Thrust/torque surface", "torque fit",
                  "RMSE %.4f" % q.get("rmse_nm", 0.0), "N*m", MEASURED,
                  "R2 %.4f; fresh-pack, no derating modelled" % q.get("r2", 0.0)))

    R.append(("Motor dynamics", "response", "%.0f" % (md.get("response_delay_s", 0) * 1000),
              "ms", MEASURED,
              "OPEN: delay or lag is not recorded, and it decides roll controllability"))
    R.append(("Motor dynamics", "model", str(md.get("model")), "-", CHOSEN,
              "the optimistic reading of the line above"))
    R.append(("Motor dynamics", "tau_s", "%.3f" % md.get("tau_s", 0.0), "s", CHOSEN,
              "first-order time constant on (T, tau_P)"))
    sl = md.get("sustained_load") or {}
    if sl:
        R.append(("Motor dynamics", "sustained derate",
                  "%.1f -> %.1f" % (sl["thrust_start_n"], sl["thrust_end_n"]), "N",
                  MEASURED, "over 7 x 60 s; recorded, NOT modelled"))

    R.append(("Environment", "gravity", "%.2f" % v.g, "m/s^2", CHOSEN, ""))
    return R


def format_parameter_table(rows=None, markdown=True):
    """-> a printable table. markdown=False gives fixed-width text for a terminal."""
    rows = rows if rows is not None else parameter_rows()
    out, section = [], None
    if markdown:
        for sec, name, val, unit, src, note in rows:
            if sec != section:
                section, _ = sec, out.extend(
                    ["", "### %s" % sec, "",
                     "| quantity | value | unit | source | note |",
                     "|---|---|---|---|---|"])
            out.append("| %s | `%s` | %s | **%s** | %s |"
                       % (name, val, unit or "-", src, note))
        return "\n".join(out).strip()
    for sec, name, val, unit, src, note in rows:
        if sec != section:
            section = sec
            out.append("")
            out.append(sec)
            out.append("-" * len(sec))

        out.append("  %-22s %14s %-9s %-10s %s"
                   % (name, val, unit or "", src, note))
    return "\n".join(out).strip()
