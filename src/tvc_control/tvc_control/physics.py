"""
physics.py -- COMPATIBILITY SHIM. The code moved; the imports still work.
================================================================================
This module used to be the whole simulator: controller, control allocation,
rigid-body dynamics, actuator models and the simulation driver, all in one file.

That was the structural problem the flight-software restructure exists to fix.
`dynamics()` never flies. `AttitudeController` does. Keeping them in one module
meant there was no way to say which lines of this project are flight software,
and therefore no way to claim that what the simulator verifies is what the
vehicle will run.

The contents now live in three packages with a one-way dependency:

    tvc_control.gnc      FLIGHT CODE -- controller, allocation, effectiveness.
                         This is what gets ported to a PX4 module.
    tvc_control.plant    SIMULATION ONLY -- rigid body, actuator lag, sensors.
                         `gnc` must never import from here.
    tvc_control.harness  Owns the clock and the transport. `mil.py` is the
                         analytic fixed-step harness.

Nothing was rewritten in the move: every symbol below was relocated verbatim and
checked back against the original text, so `sim/golden/` is unchanged by it.

This shim exists so that tvc_gui.py, tvc_view3d.py, the ROS2 nodes,
sim/validate_control.py and sim/capture_golden.py keep working untouched. Prefer
the real modules in new code -- importing `dynamics` from a package named after
flight software is exactly the confusion the split removes:

    from tvc_control.gnc.attitude import AttitudeController      # flight code
    from tvc_control.plant.rigidbody import dynamics             # simulation
    from tvc_control.harness.mil import simulate, SimConfig      # harness

References carried over from the original module:
  - de Lajarte (2021), "Guidance, Navigation and Control of a Sounding Rocket"
  - Spannagl et al. (2021), EmboRockETH gimbal geometry & cascaded PID
  - Linsen et al. (2022), Optimal TVC of an electric small-scale rocket
  - Johansen & Fossen (2013), "Control allocation -- a survey", Automatica
"""

if __package__ in (None, ""):
    # Executed as a script (`python src/tvc_control/tvc_control/physics.py`, the
    # headless smoke test SIMULATOR.md documents). Relative imports need a
    # package context, which a script does not have, so establish one. Harmless
    # when imported normally -- this branch is then not taken.
    import os as _os
    import sys as _sys

    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    __package__ = "tvc_control"

# --- flight code -------------------------------------------------------------
from .gnc.params import VehicleParams, ControlGains
from .gnc.mathx import (
    quat_normalize,
    quat_to_rotmat,
    quat_to_euler,
    euler_to_quat,
    thrust_axis,
)
from .gnc.pid import PID
from .gnc.allocation import (
    Allocation,
    axial_headroom,
    axial_limits,
    mix_motors,
    allocate,
    _lateral_gimbal,
)
from .gnc.attitude import AttitudeController
from .gnc.altitude import AltitudeController

# --- simulation --------------------------------------------------------------
from .plant.rigidbody import dynamics, quat_kinematics
from .plant.actuators import GimbalActuator

# --- harness -----------------------------------------------------------------
from .harness.mil import SimConfig, simulate, _compute_metrics

__all__ = [
    "VehicleParams", "ControlGains", "SimConfig",
    "quat_normalize", "quat_to_rotmat", "quat_kinematics", "quat_to_euler",
    "euler_to_quat", "thrust_axis",
    "PID", "Allocation", "axial_headroom", "axial_limits", "mix_motors",
    "allocate", "AttitudeController", "AltitudeController",
    "GimbalActuator", "dynamics", "simulate",
]


if __name__ == "__main__":
    # Headless smoke test -- verifies the physics/controller integrate cleanly
    # without requiring a display. Run: python3 src/tvc_control/tvc_control/physics.py
    vp = VehicleParams()
    gains = ControlGains()
    cfg = SimConfig()

    result = simulate(vp, gains, cfg)
    m = result["metrics"]
    print("Headless smoke test:")
    print(f"  final roll:  {m['final_roll_deg']:+.3f} deg (target {cfg.roll_des_deg:.1f})")
    print(f"  final pitch: {m['final_pitch_deg']:+.3f} deg (target {cfg.pitch_des_deg:.1f})")
    print(f"  max |delta1|: {m['max_delta1_deg']:.2f} deg")
    print(f"  max |delta2|: {m['max_delta2_deg']:.2f} deg")
    print(f"  settling time (pitch, 2% band): {m['settling_time_s']:.2f} s")
