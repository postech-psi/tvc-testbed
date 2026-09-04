# 7 — Roadmap

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
  [4-PARAMETERS.md](4-PARAMETERS.md).
- **Mass properties from CAD.** 1.328 kg, CG at z = 211 mm, full inertia tensor
  computed from the real mesh by tetrahedron decomposition. See
  [MASS-BUDGET.md](MASS-BUDGET.md).
- **The flight-software structure.** One controller; four pipelines; the flight
  code isolated and its porting discipline enforced by test.
- **Gazebo reproducing the measured surface exactly** via the command-side
  inversion, including its sign asymmetry.
- **A Gazebo hover.** Spawns tilted, recovers level in ~2 s, holds 2.00 m with no
  drift — flown before the restructure, with the gains that are now the default
  profile.

### Written but never run

- **The ROS 2 packages.** `colcon build` has not been executed against them.
- **The Gazebo hover, since the restructure.** The standalone demo used to carry
  its own controller and now calls the shared flight code. The gains are the
  same; that it still flies is an argument, not a measurement.
- **The PX4 airframe config** (`reference/px4/4600_tvc_coax`). PX4 is not
  installed.

---

## Next

Ordered by dependency, not by difficulty. Each one's output is the next one's
input.

### A — close the loop on what already exists

1. **First colcon build**, in the devcontainer. Everything ROS is blocked behind
   it, and it proves the toolchain before any of it can be blamed for a failure.
   Expect to resolve `ros-jazzy-actuator-msgs` and to iterate on the
   `ros_gz_bridge` type strings.
2. **Re-fly the Gazebo hover** and capture it as a golden
   (`bash gazebo/run_hover.sh --log reference/golden/hover_baseline.csv`). This
   is the largest gap in the frozen baseline.
3. **Cross-plant validation.** Run the same scenarios on the analytic plant and
   on Gazebo and compare. The machinery exists — one controller, one gain file,
   one actuator chain, two launch files differing only in the plant — but it has
   never been executed, so agreement is still an architectural claim.

   Proposed tolerances, to be ratified against the first run rather than
   defended in advance: attitude peak difference < 1.0° and RMS < 0.3°, settling
   time within 15%, peak |τ_P| within 5%, altitude within 0.05 m, final position
   within 0.10 m. They must be **larger than the divergences we know we are
   leaving in** — Gazebo's gimbal-ring and rotor gyroscopic terms, ground
   contact, bridge latency — or they are not defensible.

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

11. **PX4 SITL.** PX4 does state estimation, arming, failsafes and logging; the
    attitude loop stays external via offboard actuator control. PX4's stock
    allocator **cannot express this vehicle** — pitch and yaw come from *tilting
    one thrust vector*, not from thrust differences across fixed-direction
    rotors, and the two rotors are nearly co-located — which is why offboard is
    the plan rather than a firmware fork.

    This is where [3-CONVENTIONS.md §4](3-CONVENTIONS.md), the FRD translation
    table, becomes load-bearing. Without it, two axes out of three silently
    invert.
12. **HITL.** Real Pixhawk, simulated vehicle. Catches the timing, latency and
    serial-link problems SITL hides.
13. **The PX4 module.** The port of `gnc/` to C++, which the porting discipline
    exists to make mechanical.

### E — real flight

14. **Tethered hover.** Non-negotiable first flight. A tether that constrains
    translation but not attitude proves the attitude loop without risking the
    airframe.
15. Free hover, then translation, then landing.

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
