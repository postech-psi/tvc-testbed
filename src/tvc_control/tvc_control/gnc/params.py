"""
Vehicle and gain parameters -- the plain data the controller reads.
================================================================================
Flight code. Pure Python, no file I/O, no defaults for anything measured.

WHY VehicleParams HAS NO PHYSICAL DEFAULTS
    `VehicleParams()` used to work, filling itself from a YAML search with a
    literal fallback dict if the file could not be found. Those literals went
    stale: a superseded 20.0 N max thrust survived in three files after the
    bench measured 17.79 N, and a 180 deg/s gimbal slew outlived its measurement
    at 235/403 deg/s. Nothing failed -- the simulation just quietly described a
    different vehicle.

    So the measured fields have no defaults at all and constructing one without
    them is a TypeError. The single supported constructor is
    `tvc_control.config.load_vehicle_params()`, which reads the YAML once at
    startup and raises if it is missing. Parameters are injected, never looked
    up, which is also how they will arrive from PX4's parameter system.

    Fields that keep defaults are the ones that are choices rather than
    measurements (dx/dy disturbance terms, T_min) or genuinely optional
    (products of inertia, the measured surface, per-axis gimbal maps).
"""

import math
from dataclasses import dataclass, field


@dataclass
class VehicleParams:
    """Physical vehicle parameters.

    Angles are stored in DEGREES for the GUI's convenience and converted to
    radians via properties where the dynamics need them.

    AXIS NAMING follows the rocket convention (docs/4-CONVENTIONS.md): body x
    and y are the LATERAL axes, the two the gimbal tilts thrust about, carrying
    pitch and yaw; body z is the thrust axis, carrying ROLL. The inertia fields
    are named after the AXIS (Ix = about body x), not after the rotation, so
    they do not have to change if the naming ever does.
    """

    m: float                        # kg, total mass
    Ix: float                       # kg*m^2, about body x
    Iy: float                       # kg*m^2, about body y
    Iz: float                       # kg*m^2, about body z (thrust axis)

    L: float                        # m, ROLL lever arm: gimbal pivot -> CM
    T_max: float                    # N, max thrust (combined coax unit)
    g: float                        # m/s^2

    gimbal_max_deg: float           # deg, nominal symmetric gimbal limit
    gimbal_rate_max_deg: float      # deg/s, nominal servo slew rate
    gimbal_deadtime_s: float        # s, servo transport delay
    k_moment: float                 # m, drag-torque / thrust (analytic fallback)

    # --- products of inertia -------------------------------------------------
    # Not negligible on this airframe: Iyz/Izz = 27.3%, Ixz/Izz = 15.4%. Default
    # 0 so a caller can deliberately request the diagonal approximation, but
    # load_vehicle_params() always supplies the measured values.
    Ixy: float = 0.0
    Ixz: float = 0.0
    Iyz: float = 0.0

    # Lateral CM misalignment. OPT-IN disturbance terms, not the vehicle's
    # nominal state -- the real sub-2 mm offset lives in the SDF link pose and
    # the products of inertia above.
    dx: float = 0.0
    dy: float = 0.0

    T_min: float = 5.0              # N, idle floor; must stay > 0 for allocation

    # --- aerodynamics (ESTIMATED, opt-in) ------------------------------------
    # There is NO aero bench data. These are design-intent estimates so a
    # scenario can ask "does plausible drag change a conclusion?"; conclusions
    # resting on them are provisional. Default OFF so the plant is unchanged and
    # the frozen baselines reproduce. See plant/aero.py.
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
    """Cascaded PID gains: attitude loop outside, rate loop inside.

    LATERAL (body x, y) gains are shared between the two axes -- Ix and Iy
    differ by 0.15% on this airframe, so treating them as symmetric is exact to
    within the mass-budget uncertainty.

    ROLL (body z) gains are SEPARATE and much smaller, and this is not a tuning
    preference -- it is forced by the airframe. Iz = 0.00196 kg*m^2 is 11.6x
    smaller than Ix, so the same torque produces 11.6x the angular acceleration
    about z. Sharing one gain set makes the roll channel violently underdamped
    (or the lateral channel uselessly slow). The two channels also have
    different actuators and therefore different lags: the lateral axes go
    through a 30 ms servo deadtime, the roll channel through the motor time
    constant (~100 ms measured), which is why roll authority being larger does
    not mean the roll loop can be faster.

    THE DEFAULTS HERE ARE NOT THE FLOWN SET. They are the analytic simulator's
    historical values, kept so that pre-existing plots and the frozen baseline
    reproduce. The set that has actually flown is the `flight_validated` profile
    in control_gains.yaml, which is the file default and what every pipeline
    loads. Construct ControlGains directly only when you specifically want these.
    """

    # UNITS: the rate loop outputs ANGULAR ACCELERATION [rad/s^2], not torque.
    # Inertia is applied once, in AttitudeController. The values below are the
    # historical torque-unit gains divided by their axis inertia, so behaviour is
    # unchanged; what changed is that they became comparable with the flown set,
    # which was always written this way.
    kp_angle: float = 4.0      # attitude loop: angle error -> rate setpoint
    kp_rate: float = 0.8843296781     # rate loop: rate error -> angular accel
    ki_rate: float = 0.0884329678
    kd_rate: float = 0.1768659356
    i_limit: float = 22.1082419526     # integrator clamp (anti-windup), rad/s^2

    # --- roll channel (body z / tau_P) ---
    # No longer carries a hand-baked Iz/Ix factor: the inertia scaling is
    # explicit in the controller, so re-measuring Iz changes the torque these
    # produce without silently changing the loop bandwidth.
    kp_angle_roll: float = 4.0
    kp_rate_roll: float = 0.9197751661
    ki_rate_roll: float = 0.0919775166
    kd_rate_roll: float = 0.1788451712
    i_limit_roll: float = 25.5493101686

    # --- altitude cascade: outer P (m -> m/s), inner PID (m/s -> m/s^2) ---
    kp_alt: float = 1.5        # m/s of climb demand per m of altitude error
    kp_vz: float = 4.0         # m/s^2 per m/s
    ki_vz: float = 1.0
    kd_vz: float = 0.2
    i_limit_vz: float = 4.0    # m/s^2 of integral authority
    vz_max: float = 2.0        # m/s, climb/descent rate limit

    # --- position hold: horizontal error -> commanded tilt ---
    # Deliberately ~10x slower than the attitude loop; see gnc/position.py for
    # why closing that gap produces a coning limit cycle rather than a faster
    # response.
    kp_pos: float = 0.10       # rad of tilt per m of position error
    kd_pos: float = 0.20       # rad per m/s
    max_tilt_deg: float = 8.0

    @property
    def max_tilt(self):
        """Largest tilt the position loop may command, in radians."""
        return math.radians(self.max_tilt_deg)
