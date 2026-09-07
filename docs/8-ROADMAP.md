# 8 — Roadmap

Where this is going, what is deferred, and what is genuinely unknown. The last
category is the one worth reading.

**The goal** is a demonstrator that takes off vertically, hovers under thrust
vector control, and lands — a testbed for reusable-launch-vehicle GNC, built so
that the next team of students can extend it rather than rebuild it. POSTECH
UGRP 2026.

---

## The two repositories

They are not arbitrary halves; they answer different questions.

| repo | question | contains |
|---|---|---|
| **`tvc-data`** | *what does the hardware actually do?* | bench measurements and the analysis pipeline that turns raw logs into a thrust/torque map |
| **`tvc-testbed`** | *what will the vehicle do, and what code flies it?* | this repository: flight code, physics model, Gazebo simulation, CAD-derived mass properties |

The dependency runs one way and should stay that way:

```
real hardware ──measured on the bench──▶ tvc-data ──parameterises──▶ tvc-testbed ──deployed to──▶ real vehicle
```

**Never put simulation output in `tvc-data`.** It is the record of physical
measurement; mixing predictions into it destroys its only advantage.

---

## Where we are

### Done

- **Bench characterisation.** A 121-point coax thrust/torque surface with stated
  fit error, per-ring gimbal maps with travel, slew, bandwidth, hysteresis and
  deadtime, and a measured voltage-sag law. See
  [5-PARAMETERS.md](5-PARAMETERS.md).
- **Mass properties from CAD.** 1.328 kg, CG at z = 211 mm, full inertia tensor
  computed from the real mesh by tetrahedron decomposition. See
  [MASS-BUDGET.md](MASS-BUDGET.md).
- **The flight-software structure.** One controller; four pipelines; the flight
  code isolated behind a small, documented interface.
- **Gazebo reproducing the measured surface exactly** via the command-side
  inversion, including its sign asymmetry.
- **The Gazebo hover, under the shared flight code.** From the world's
  deliberate 10°/−7° spawn: level in **1.03 s**, 2.000 m held, 1 × 10⁻⁸ m of
  drift, thrust settling at exactly *mg*. Frozen as
  `reference/golden/hover_baseline.json` and checked by
  `tvc.py hover --check-golden`; the picture is
  [hover_baseline.png](../reference/golden/hover_baseline.png).
- **The ROS 2 stack, running.** Both packages build, all four long-running
  processes start, and messages cross `ros_gz_bridge` in both directions with the
  vehicle holding altitude and lateral attitude. Two startup bugs that a clean
  `colcon build` did not catch are fixed; see
  [7-CREDIBILITY.md](7-CREDIBILITY.md).
- **Graphs and the GUI, working.** `tvc.py plot` renders either plant through one
  layout, `tvc.py validate --plot DIR` renders all five analytic scenarios,
  `tvc.py gui` drives the analytic harness from a form, and `tvc.py view3d`
  animates a run on the real CAD mesh.

### The open questions

- **The ROS 2 path holds a 1.5–2.5° roll limit cycle the direct path does not.**
  Two SIL pipelines, identical control code, agreeing on every channel except
  the thrust axis — the one with 11.5× less inertia and a ~3× slower actuator,
  where the bridge's transport delay costs phase margin. Bounded and
  attributable; not explained in detail. **This is the highest-value open item:
  it is the cross-plant disagreement that cross-plant validation exists to
  find, and it is now small enough to study rather than large enough to
  obscure everything else.**
- **The vehicle cannot hold roll attitude in a full-throttle climb.** Above
  ~17.0 N the feasible τ_P interval no longer contains zero, so a forced
  +0.017 N·m roll torque is applied and the thrust axis takes 53° before the
  throttle comes off the stop. Real behaviour, correctly modelled, with an
  operational consequence — a climb profile that stays below ~95% throttle keeps
  roll authority, and nothing currently enforces that.
- **The 100 ms motor response is still not identified as a delay or a lag.**
  Unchanged, and still the highest-value single bench measurement outstanding:
  it decides whether the roll channel is controllable at all.

The analytic ROS 2 pipeline (`analytic.launch.py`) runs too: attitude and
altitude hold exactly, and the vehicle translates away at a constant velocity
because that launch file leaves `position_hold` false and the world's initial
tilt imparts one. Correct, and worth knowing before someone reports it as drift.

### Still never run

- **Anything on hardware.** No line of this code has driven a servo or an ESC,
  and nothing here has been compared against flight data.

---

## Next

Ordered by dependency, not by difficulty. Each one's output is the next one's
input.

### A — close the loop on what already exists

1. **Explain and remove the ROS 2 thrust-axis limit cycle.** Reproduce Pipeline
   2 and Pipeline 4 with timestamped command/state logs, then separate bridge
   latency, sample timing, and actuator-model differences one at a time. The
   exit condition is either agreement with the direct path or a measured,
   documented transport requirement.
2. **Automate matched analytic-versus-Gazebo scenarios.** Replay the same
   initial states and setpoints on both plants and compare attitude peak/RMS,
   settling time, peak |τ_P|, altitude, and final position. Ratify tolerances
   from repeated runs; they must cover the known Gazebo gyroscopic, contact, and
   timing effects without hiding a sign, frame, or allocation error.
3. **Protect the roll-authority envelope.** Add a command governor or climb
   profile constraint that preserves a feasible τ_P interval around zero. The
   full-throttle scenario should fail explicitly instead of silently accepting
   a forced roll torque.

### B — the measurements that would change conclusions

4. **Is the 100 ms motor response a delay or a lag?** One bench run. It decides
   whether the roll channel is controllable at the current gains. This is the
   highest-value outstanding item in the entire project.
5. **Knife-edge CG measurement** of the assembled vehicle. `L` sets all lateral
   authority and is currently an assumption resting on assumed electronics
   positions.
6. **Fill the off-diagonal coax cells.** 36 of 72 cells are measured; A ∈ {1700,
   1800, 1850} are diagonal-only. Interpolated cells are flagged as extrapolated
   — they are guesses, not data.
7. **Held-out validation of the surface.** The fit used 119 of 121 points and
   reserved none.

### C — modelling work the seams are already waiting for

8. **Takeoff and landing.** Hover starts airborne today. Ground contact during
   spin-up is its own problem: the legs are the only contact and the vehicle is
   tall and narrow. Landing needs touchdown detection and thrust cutoff.
9. **Disturbance and robustness.** Wind, sensor noise, thrust mismatch between
   rotors, ±20–30% inertia error. If the gains only work at the nominal mass
   properties, they will not survive the real vehicle.
10. **Uncertainty propagation.** Perturb `L`, mass and the surface by their
    stated uncertainty; report the spread of the scenario metrics. This is a
    loop over existing code and it would move Results Uncertainty off 0.

### D — the flight stack

11. **Define stale-data behaviour before any hardware-in-the-loop run.** The
    current ROS controller is SIL-only: it has no odometry-age check, and the
    actuator interface has no command timeout. Specify the maximum state age,
    the transport watchdog, and the disarmed output, then verify that dropping
    either odometry or commands removes thrust instead of holding the last
    command.
12. **PX4 SITL.** PX4 does state estimation, arming, failsafes and logging; the
    attitude loop stays external via offboard actuator control. PX4's stock
    allocator **cannot express this vehicle** — pitch and yaw come from *tilting
    one thrust vector*, not from thrust differences across fixed-direction
    rotors, and the two rotors are nearly co-located — which is why offboard is
    the plan rather than a firmware fork.

    This is where [4-CONVENTIONS.md §4](4-CONVENTIONS.md), the FRD translation
    table, becomes load-bearing. Without it, two axes out of three silently
    invert.
13. **HITL.** Real Pixhawk, simulated vehicle. Catches the timing, latency and
    serial-link problems SITL hides.
14. **The PX4 module.** The port of `gnc/` to C++, which the porting discipline
    exists to make mechanical.

### E — real flight

15. **Tethered hover.** Non-negotiable first flight. A tether that constrains
    translation but not attitude proves the attitude loop without risking the
    airframe.
16. Free hover, then translation, then landing.

---

## Deferred, with the data already in the YAML

Re-enabling any of these is a **code change, not a re-measurement** — the numbers
are recorded in `vehicle_params.yaml` next to the rest of the vehicle.

| deferred | the data waiting for it |
|---|---|
| servo resonance | `gimbal.axes.*.resonance_db`, `phase_lag_2hz_deg` |
| battery sag and thrust derating | `motor_dynamics.sustained_load`, `thrust_sensitivity_n_per_v` |
| gimbal hysteresis | `gimbal.axes.*.fit.hysteresis_*_deg` |
| sensor noise and an estimator | none needed — **Seam A is already in place**, so this drops in without touching the controller |
| an electrical model | `motor_dynamics.peak_current_a`, `efficiency_g_per_w` |

---

## What is genuinely uncertain

Ranked by how much damage each does if wrong.

1. **Whether the motor response is a delay or a lag.** As a lag the roll channel
   is fine; as a delay it is not controllable at the current gains. Nothing else
   on this list can flip a conclusion this completely, and one bench run settles
   it.
2. **Electronics positions.** 748 g — over half the vehicle — sits at *assumed*
   locations. CG and inertia both depend on them, and the gains depend on those.
3. **The lever-arm convention.** `pivot` versus `rotor_plane` is worth 21% of all
   lateral authority and the decision is still deferred.
4. **The battery swap.** 4200 mAh → 2200 mAh removes ~130–150 g from z = 306 mm,
   near the top. That moves the CG down and cuts pitch/yaw inertia noticeably.
   Re-derive after the swap; do not scale.
5. **T/W = 1.37.** Workable, not generous. Every gram added eats margin, and
   thrust falls 13% as the pack drains.
6. **Roll authority is weak and thrust-dependent.** Rotor differential only, it
   trades against total thrust, and it collapses toward zero at the ceiling.
   There is no gimbal authority about that axis at all.
7. **Off-diagonal coax data.** Half the map's cells are interpolated.
8. **Aerodynamics.** Not modelled at all. Ground effect during landing is the
   most likely to bite.

---

## The actual deliverable

The point is not one flight; it is a platform. That means the mass budget, the
thrust map and the control architecture stay documented and **re-derivable** when
the next team changes the airframe — which is why every number is generated from
one file and every convention is asserted by a test rather than written in a
comment.
