"""Physical parameters and control gains passed into the controller at startup."""

import math
from dataclasses import dataclass, field


@dataclass
class VehicleParams:
    """Physical vehicle parameters.

    Angles are stored in DEGREES for the GUI's convenience and converted to
    radians via properties where the dynamics need them.

    AXIS NAMING follows the rocket convention (docs/GUIDE.md): body x
    and y are the LATERAL axes, the two the gimbal tilts thrust about, carrying
    pitch and yaw; body z is the thrust axis, carrying ROLL. The inertia fields
    are named after the AXIS (Ix = about body x), not after the rotation, so
    they do not have to change if the naming ever does.
    """

    m: float                        # kg, total mass
    Ix: float                       # kg*m^2, about body x
    Iy: float                       # kg*m^2, about body y
    Iz: float                       # kg*m^2, about body z (thrust axis)

    L: float                        # m, TVC lever arm: gimbal pivot -> CM
    T_max: float                    # N, max thrust (combined coax unit)
    g: float                        # m/s^2

    gimbal_max_deg: float           # deg, nominal symmetric gimbal limit
    gimbal_rate_max_deg: float      # deg/s, nominal servo slew rate
    gimbal_deadtime_s: float        # s, servo transport delay
    k_moment: float                 # m, drag-torque / thrust (analytic fallback)

    # --- products of inertia -------------------------------------------------
    # Not negligible on this airframe: Iyz/Izz = 27.3%, Ixz/Izz = 15.4%. Default
    # 0 so a caller can deliberately request the diagonal approximation, but
    # load_vehicle_params() supplies the configured values.
    Ixy: float = 0.0
    Ixz: float = 0.0
    Iyz: float = 0.0

    # Lateral CM misalignment. OPT-IN disturbance terms, not the vehicle's
    # nominal state. The configured CG and products of inertia remain separate.
    dx: float = 0.0
    dy: float = 0.0

    T_min: float = 5.0              # N, idle floor; must stay > 0 for allocation

    # --- aerodynamics (ESTIMATED, opt-in) ------------------------------------
    # There is NO aero bench data. These are design-intent estimates so a
    # scenario can ask "does plausible drag change a conclusion?"; conclusions
    # resting on them are provisional. Default OFF so the plant is unchanged and
    # the frozen baselines reproduce. See physics/aero.py.
    aero_enabled: bool = False
    cd: float = 0.0                 # drag coefficient (dimensionless)
    ref_area: float = 0.0           # m^2, reference frontal area
    rho: float = 1.225              # kg/m^3, air density
    rotor_radius: float = 0.0       # m, for the ground-effect model

    # Roll (thrust-axis) reaction torque authority. tau_p_max is an optional
    # measured hard cap; when None the allocator reads the feasible set off the
    # measured surface instead.
    tau_p_max: float = None

    # Bench-measured (PWM A, PWM B) -> (thrust, tau_P) surface, or None. When
    # present it REPLACES the k_moment model everywhere authority is computed;
    # k_moment survives only for the Gazebo plugin, which cannot take a surface.
    surface: object = None
    # Per-ring measured gimbal maps {'inner': .., 'outer': ..}, or None.
    gimbal_axes: object = field(default=None)

    @property
    def gimbal_max(self):
        """Nominal symmetric gimbal limit, in radians."""
        return math.radians(self.gimbal_max_deg)

    @property
    def gimbal_rate_max(self):
        """Nominal servo slew rate (the SLOWER ring), in rad/s."""
        return math.radians(self.gimbal_rate_max_deg)

    # --- per-ring travel -----------------------------------------------------
    # delta1 is the yaw-plane deflection, carried by the INNER ring; delta2 is
    # the pitch-plane one, carried by the OUTER ring (base_link -> pitch joint ->
    # outer ring -> yaw joint -> inner ring -> rotors). Neither ring is
    # symmetric about its own neutral and they differ from each other, so the
    # limits are per-axis rather than one scalar. Falls back to the symmetric
    # +/-gimbal_max_deg when the measured block is absent.

    def _axis(self, name):
        return (self.gimbal_axes or {}).get(name)

    @property
    def delta_min(self):
        """Most negative travel per ring, (inner, outer), in radians."""
        inner, outer = self._axis("inner"), self._axis("outer")
        return (math.radians(inner.min_deg if inner else -self.gimbal_max_deg),
                math.radians(outer.min_deg if outer else -self.gimbal_max_deg))

    @property
    def delta_max(self):
        """Most positive travel per ring, (inner, outer), in radians."""
        inner, outer = self._axis("inner"), self._axis("outer")
        return (math.radians(inner.max_deg if inner else self.gimbal_max_deg),
                math.radians(outer.max_deg if outer else self.gimbal_max_deg))

    @property
    def delta_rate_max(self):
        """Slew limit per ring, (inner, outer), in rad/s."""
        inner, outer = self._axis("inner"), self._axis("outer")
        return (math.radians(inner.rate_max_deg if inner else self.gimbal_rate_max_deg),
                math.radians(outer.rate_max_deg if outer else self.gimbal_rate_max_deg))


@dataclass
class ControlGains:
    """Cascaded controller gains loaded from settings/gains.yaml."""

    # Rate PID outputs angular acceleration; AttitudeController applies inertia.
    kp_angle: float
    kp_rate: float
    ki_rate: float
    kd_rate: float
    i_limit: float

    # --- roll channel (body z / tau_P) ---
    # No longer carries a hand-baked Iz/Ix factor: the inertia scaling is
    # explicit in the controller, so re-measuring Iz changes the torque these
    # produce without silently changing the loop bandwidth.
    kp_angle_roll: float
    kp_rate_roll: float
    ki_rate_roll: float
    kd_rate_roll: float
    i_limit_roll: float

    # --- altitude cascade: outer P (m -> m/s), inner PID (m/s -> m/s^2) ---
    kp_alt: float
    kp_vz: float
    ki_vz: float
    kd_vz: float
    i_limit_vz: float
    vz_max: float

    # --- position hold: horizontal error -> commanded tilt ---
    # Deliberately ~10x slower than the attitude loop; see control/position.py for
    # why closing that gap produces a coning limit cycle rather than a faster
    # response.
    kp_pos: float
    kd_pos: float
    max_tilt_deg: float

    @property
    def max_tilt(self):
        """Largest tilt the position loop may command, in radians."""
        return math.radians(self.max_tilt_deg)
