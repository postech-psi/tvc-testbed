"""Load manually maintained vehicle settings and controller gains. Units become SI here."""
import os
from pathlib import Path
from dataclasses import dataclass

import yaml

_DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "settings", "vehicle.yaml")

# The axis convention the YAML (and therefore every consumer) is written in.
# See docs/GUIDE.md. Loading a file that predates or postdates the rename
# must fail loudly rather than reinterpret its numbers under the wrong names.
SUPPORTED_AXIS_CONVENTIONS = ("rocket_v2",)


def gazebo_directory():
    """Find Gazebo assets in a source checkout or a colcon installation."""
    source = Path(__file__).resolve().parents[1] / "gazebo"
    if source.is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("tvc_control")) / "gazebo"


@dataclass
class Vehicle:
    """Flattened, unit-normalized view of settings/vehicle.yaml.

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
    """Parse settings/vehicle.yaml into a Vehicle. Raises on a missing/invalid
    lever_arm_mode rather than silently guessing a lever arm."""
    path = path or _DEFAULT_PATH
    if not os.path.isfile(path):
        raise FileNotFoundError(
            "Vehicle settings not found: %s" % path)
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)

    conv = d.get("axis_convention", "rocket_v2")
    if conv not in SUPPORTED_AXIS_CONVENTIONS:
        raise ValueError(
            "unknown axis_convention %r in %s -- see docs/GUIDE.md"
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
    from .control.params import VehicleParams
    from .control.effectiveness import ThrustTorqueSurface, GimbalAxisMap

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

    aero = v.raw.get("aero") or {}

    return VehicleParams(
        m=v.mass, Ix=v.Ix, Iy=v.Iy, Iz=v.Iz,
        Ixy=v.Ixy, Ixz=v.Ixz, Iyz=v.Iyz,
        L=v.L,
        aero_enabled=bool(aero.get("enabled", False)),
        cd=float(aero.get("cd", 0.0)),
        ref_area=float(aero.get("ref_area_m2", 0.0)),
        rho=float(aero.get("air_density", 1.225)),
        rotor_radius=float(aero.get("rotor_radius_m", 0.0)),
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
                           "settings", "gains.yaml")


def load_gains(profile=None, path=None):
    """Build ControlGains from a named profile in settings/gains.yaml.

    profile=None takes the file's own default_profile, so switching the whole
    project between gain sets is a one-line edit in one file rather than a
    search for constructor call sites.
    """
    from .control.params import ControlGains

    path = path or _GAINS_PATH
    if not os.path.isfile(path):
        raise FileNotFoundError("settings/gains.yaml not found at %s" % path)
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
