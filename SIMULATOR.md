# TVC 6-DOF Attitude Simulator — Interactive GUI

Polished Step 2 simulator with an interactive desktop GUI: edit vehicle
parameters, control gains, and target/disturbance conditions, click **Run
Simulation**, and see the attitude/rate/gimbal response update immediately in
embedded plots.

## ROS2 / Gazebo development environment

Phases 3–5 of this project (ROS2, Gazebo SIL testing, PX4 SITL bridging) run
inside a Docker dev container so the whole team shares an identical setup.
See [README.md](README.md) to get started — it only takes a few clicks once
Docker Desktop and VS Code are installed.

## Files

- `tvc_physics.py` — physics core (quaternion kinematics, Newton-Euler
  dynamics, gimbal actuator model, cascaded PID controller, simulation
  driver). Runnable standalone as a headless smoke test:
  ```
  python3 tvc_physics.py
  ```
- `tvc_gui.py` — the interactive Tkinter GUI. Run this for normal use:
  ```
  python3 tvc_gui.py
  ```

## Requirements

```
pip install -r requirements.txt
```

`tkinter` is required and is normally bundled with the standard Python
installer on Windows/macOS. On Debian/Ubuntu Linux, if `python3 -c "import
tkinter"` fails, install it with:

```
sudo apt-get install python3-tk
```

## What you can adjust in the GUI

**Vehicle Parameters** — mass, roll/pitch/yaw inertia, axial TVC lever arm
`L` (gimbal pivot → CM, per the corrected physics — NOT a lateral offset),
and optional lateral CM-misalignment disturbance terms `dx`/`dy`.

**Actuator Limits** — max/min thrust, gimbal mechanical limit (deg), gimbal
servo slew rate (deg/s).

**Cascaded PID Gains** — outer-loop angle gain, inner-loop rate PID gains,
and the rate-loop integrator anti-windup clamp.

**Simulation / Target** — sim duration, control period, target roll/pitch,
and initial roll/pitch disturbance (the "3° roll / -4° pitch → 0°/+5° target"
style test case from Step 2 is the default).

## Reading the output

Three stacked plots:

1. **Attitude Response** — roll/pitch Euler angles vs. time, with dashed
   lines marking the target setpoints.
2. **Body Angular Velocity** — ω_x, ω_y, ω_z in deg/s.
3. **Gimbal Command** — δ₁ (pitch-plane) and δ₂ (roll-plane) gimbal angles,
   with dotted lines at the configured mechanical limit.

A metrics panel below the control form reports final roll/pitch error, 2%-
band settling time (on pitch), and max gimbal deflection with percent-of-
limit utilization — including a warning if the gimbal is saturating (>90%
of its mechanical limit), which usually means the disturbance is too large
or the gains need retuning.

## Physics notes carried over from Step 1 / de Lajarte Ch. 4

- The TVC moment arm is the **axial** distance `L` from the gimbal pivot to
  the vehicle center of mass (along body +z) — not a lateral CM offset. A
  lateral offset crossed with an on-axis thrust vector gives zero torque to
  leading order; `dx`/`dy` are kept only as optional disturbance terms.
- Roll/yaw about the thrust axis (body z) has no gimbal authority — real
  yaw control comes from differential RPM between the coax motors, not
  modeled in this attitude-only simulator.
- Attitude is propagated with quaternions throughout (never Euler angles
  internally) to avoid gimbal lock; Euler angles are computed only for
  readout/plotting.
