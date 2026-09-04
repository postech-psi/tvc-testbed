> **Restructured.** The simulator was rebuilt as a flight-software development
> harness: flight code, plant and harness are now separate packages, the flight
> code is pure-Python by enforced rule so the PX4 port is mechanical, and the
> axis convention is unified (roll = thrust axis). Start at
> [CONVENTIONS.md](CONVENTIONS.md); the honest account of what is and is not
> verified is [CREDIBILITY.md](CREDIBILITY.md).
>
> **The highest-value measurement outstanding** is whether the bench's 100 ms
> motor response is a transport delay or a first-order lag. Modelled both ways:
> as a lag the roll channel recovers cleanly; as a delay it winds up to ~70 deg
> and saturates 97% of the run. One bench run decides whether that channel is
> controllable at the current gains.
>
> Deferred, with the data already in `vehicle_params.yaml` so re-enabling is a
> code change and not a re-measurement: servo resonance (`resonance_db`,
> `phase_lag_2hz_deg`), battery sag and thrust derating
> (`motor_dynamics.sustained_load`), sensor noise and an estimator (Seam A is
> already in place, so this drops in without touching the controller),
> takeoff/landing, PIL and HIL, PX4 SITL.

# TVC VTVL — where this is going

POSTECH UGRP 2026. The goal is a **demonstrator that takes off vertically,
hovers under thrust vector control, and lands** — a testbed for
reusable-launch-vehicle GNC, built so later students can extend it rather than
rebuild it.

This document is the map: what exists, what it depends on, and what is
genuinely unknown. It is deliberately blunt about the last category.

## The two repositories

They are not arbitrary halves; they answer different questions.

| Repo | Question it answers | Contains |
|---|---|---|
| **`tvc-data`** | *What does the hardware actually do?* | Bench measurements, the analysis pipeline that turns raw logs into a thrust/torque map |
| **`tvc-testbed`** | *What will the vehicle do, and what code flies it?* | Physics model, Gazebo simulation, control code, CAD-derived mass properties |

The dependency runs one way and should stay that way:

```
   real hardware
        │  measured on the bench
        ▼
   tvc-data  ── thrust/torque map, voltage sag, max thrust
        │
        │  parameterises
        ▼
   tvc-testbed ── Gazebo model + control gains
        │
        │  deployed to
        ▼
   real vehicle
```

**Never put simulation output in `tvc-data`.** It is the record of physical
measurement; mixing predictions into it destroys its only advantage.

## Where we are

### Done and verified

- **Bench characterisation.** Coax thrust/torque map over (A, B) commands, with
  honest autocorrelation-corrected error bars. Voltage-sag law measured
  (thrust ∝ V^1.26), so thrust can be quoted at a stated pack voltage instead
  of whatever the battery happened to be at.
- **Max thrust: 20 N**, T/W ≈ 1.54. From the 2026-07-20 runs, the only ones
  reaching full throttle. Cross-checked against an independent extrapolation of
  the 07-24 coax data to ~1.5%.
- **Mass properties from CAD.** m = 1.328 kg, CG at z = 211 mm, inertia tensor
  from the real mesh. See [MASS_BUDGET.md](MASS_BUDGET.md).
- **Gazebo hover.** The vehicle spawns tilted (roll +10°, pitch −7°), recovers
  level in ~2 s, holds 2.00 m with zero drift. Thrust vectoring via a real
  kinematic chain; yaw via rotor differential. See [../sim/README.md](../sim/README.md).

### Written but never run

- ROS2 nodes (`controller_node`, `gazebo_bridge_node`, `gazebo.launch.py`).
  `hover.py` bypasses them by talking to Gazebo directly.
- PX4 airframe config (`sim/px4/4600_tvc_coax`). PX4 itself is not installed.
- `tvcbench` acquisition rewrite — fully tested against simulated sources,
  never against hardware.

## The roadmap

Phases are ordered by dependency, not by difficulty. Each one's output is the
next one's input.

### Phase A — Close the simulation loop *(nearest term)*

1. **Takeoff and landing.** Hover starts airborne today. Ground contact during
   spin-up is its own problem: the legs are the only contact, and the vehicle
   is tall and narrow. Landing needs touchdown detection and thrust cutoff.
2. **Disturbance and robustness testing.** Wind, sensor noise, thrust
   mismatch between rotors, ±20–30 % inertia error. If the gains only work at
   the nominal mass properties, they will not survive the real vehicle.
3. **ROS2 path.** Run the nodes that already exist, so the architecture that
   ships to hardware is the one that was tested — not `hover.py`.

### Phase B — Flight-stack integration

4. **PX4 SITL.** PX4 does state estimation, arming, failsafes, logging; the
   attitude loop stays in ROS2 via offboard actuator control. PX4's stock
   allocator cannot express this vehicle (pitch/roll come from *tilting one
   thrust vector*, not from thrust differences), which is why offboard is the
   plan rather than a firmware fork.
5. **HITL.** Real Pixhawk hardware, simulated vehicle. This is what catches
   timing, latency and serial-link problems that SITL hides.

### Phase C — Real flight

6. **Tethered hover.** Non-negotiable first flight. A tether that constrains
   translation but not attitude proves the attitude loop without risking the
   airframe.
7. **Free hover, then translation, then landing.**

### Phase D — Extensibility (the actual UGRP deliverable)

The point is not one flight; it is a platform. That means the mass budget,
thrust map, and control architecture stay documented and re-derivable when the
next team changes the airframe.

## What is genuinely uncertain

Ranked by how much damage each does if wrong.

1. **Electronics positions.** 748 g — over half the vehicle — sits at *assumed*
   locations. CG and inertia both depend on them, and the gains depend on
   those. **Fix:** balance the assembled vehicle on a knife edge, set
   `center_of_mass_override_mm` in `tools/components.yaml`.
2. **The battery swap.** 4200 mAh → 2200 mAh removes ~130–150 g from z = 306 mm,
   near the top. That moves CG down and cuts pitch/roll inertia noticeably.
   Re-derive after the swap; do not scale.
3. **T/W = 1.54.** Workable, not generous. Every gram added eats margin, and
   thrust falls as the pack drains (measured: 13.8 % across a discharge).
4. **Yaw authority is weak.** Rotor differential only, and it trades against
   total thrust. There is no gimbal authority in yaw at all.
5. **Off-diagonal coax map.** 36 of 72 cells measured; A ∈ {1700, 1800, 1850}
   are diagonal-only. `tvctools interp` fills gaps but flags them as
   extrapolated — they are guesses, not data.
6. **Sim fidelity.** Rotor aerodynamics are a momentum-theory approximation;
   ground effect, blade flapping and wind are not modelled.

## Immediate next actions

| # | Action | Blocks | Cost |
|---|---|---|---|
| 1 | **Commit everything.** `sim/`, `tools/`, `docs/`, `src/tvc_control/` are all untracked. | everything | minutes |
| 2 | Knife-edge CG measurement on the assembled vehicle | gain validity | hours |
| 3 | Takeoff + landing in sim | Phase B | days |
| 4 | Fill the off-diagonal coax cells on the bench | torque model | one session |
| 5 | Install PX4, run SITL | Phase B | ~1 day |

Items 2 and 4 are hardware-side and can run in parallel with 3 and 5.
