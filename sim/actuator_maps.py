"""
sim/actuator_maps.py -- COMPATIBILITY SHIM.
================================================================================
ThrustTorqueSurface and GimbalAxisMap are FLIGHT CODE -- the allocator needs the
measured surface to know its own feasible set -- so they moved into
tvc_control/gnc/effectiveness.py and were rewritten without numpy.

The old ThrustMap / ServoMap classes are gone. They wrapped the `pwm_thrust_map`
and `pwm_servo_map` YAML blocks, which were never populated (both still `type:
null`) and are superseded by the measured two-input surface. Nothing imported
them.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "tvc_control"))

from tvc_control.gnc.effectiveness import (  # noqa: E402,F401
    ThrustTorqueSurface, GimbalAxisMap,
)
from tvc_control.config import load, load_vehicle_params  # noqa: E402,F401


if __name__ == "__main__":
    v = load()
    vp = load_vehicle_params()
    surf = vp.surface
    print("measured surface: thrust %.2f .. %.2f N" % surf.thrust_limits())
    print("  roll torque available vs thrust:")
    for frac in (0.50, 0.65, 0.75, 0.85, 0.95):
        T = surf.thrust_max * frac
        lo, hi = surf.torque_limits_at(T)
        print("    T = %5.2f N (%3.0f%%)  ->  tau_P in [%+.4f, %+.4f] N.m"
              % (T, 100 * frac, lo, hi))
    lo, hi = surf.torque_limits_at(v.weight_n)
    print("  hover  T = %.2f N (%3.0f%%)  ->  tau_P in [%+.4f, %+.4f] N.m"
          % (v.weight_n, 100 * v.weight_n / surf.thrust_max, lo, hi))
    a, b, T, Q = surf.inverse(v.weight_n, 0.10)
    print("  inverse(T=%.2f, tau_P=0.100) -> A=%.0f us, B=%.0f us "
          "(achieved %.4f N, %.4f N.m)" % (v.weight_n, a, b, T, Q))
    print("")
    for name, ax in (vp.gimbal_axes or {}).items():
        print("gimbal %-5s: %+.2f..%+.2f deg, neutral %.1f us, "
              "1600 us -> %+.2f deg, %.0f deg/s, BW %.0f Hz"
              % (name, ax.min_deg, ax.max_deg, ax.neutral,
                 ax.pwm_to_deg(1600.0), ax.rate_max_deg, ax.bandwidth_hz))
