# 7 — Credibility

How much this simulator's output should be trusted, and on what evidence.

Structured after **NASA-STD-7009**, *Standard for Models and Simulations*, which
assesses credibility on eight factors in three categories and requires that
results reaching a decision-maker carry explicit caveats, an uncertainty
estimate, and the assessment itself — rather than arriving as bare numbers.

> **Scope of the borrowing.** The eight factor names and the three categories are
> the standard's. The 0–4 level *anchors* below are project-local: written for
> this vehicle and this team, not lifted from the standard's own definitions.
> Treat the numbers as a shared vocabulary for "how much evidence is behind
> this", not as a NASA-conformant assessment.
>
> **Blank is a real answer.** A factor with no evidence is recorded as 0 and left
> visibly empty. The point of this file is to convert unexamined confidence into
> stated ignorance, so filling a cell to make the table look finished defeats it.

**Levels.** 0 no evidence · 1 informal or anecdotal · 2 documented but unverified
· 3 verified against an independent reference · 4 verified plus quantified
uncertainty.

---

## Summary

| category | factor | level | basis |
|---|---|---|---|
| M&S Development | Verification | **3** | a unit suite, 5 closed-loop scenarios and a frozen baseline, all gated in CI |
| M&S Development | Validation | **0** | *nothing has been compared against the real vehicle* |
| M&S Operations | Input Pedigree | **3** | actuators bench-measured with stated fit error; mass properties are CAD plus assumptions |
| M&S Operations | Results Uncertainty | **0** | measurement uncertainty is recorded and never propagated |
| M&S Operations | Results Robustness | **2** | divergences enumerated, one quantified; no sensitivity study |
| Supporting Evidence | Use History | **2** | analytic scenarios, direct Gazebo hover, and both ROS plant paths have run; repetition is not automated |
| Supporting Evidence | M&S Management | **4** | single source of truth, three generated artefacts, all `--check`ed in CI |
| Supporting Evidence | People Qualifications | — | not assessed |

> ### The headline caveat, to be repeated wherever results are shown
>
> **This simulator has never been compared against flight data.** It reproduces
> bench-measured **actuator** behaviour; it has not been shown to reproduce
> **vehicle** behaviour. Every settling time, every margin and every authority
> claim in this repository is a statement about a model, not about an aircraft.

---

## M&S Development

### Verification — "was the model built right?" · level 3

**What exists.**

- **The unit suite** (`tests/`): pure Python, no ROS or Gazebo, seconds to run.
  Both the counts and the descriptions below are generated from the test files
  themselves, so this table cannot go stale.

<!-- <<<EMIT:tests -->
| file | test functions | what it guards |
|---|---|---|
| `test_allocation.py` | 7 | Control allocation: does it realize the moment it was asked for? |
| `test_attitude_error.py` | 8 | The quaternion attitude error, and why it replaced the Euler difference. |
| `test_axis_convention.py` | 9 | The axis convention, asserted rather than documented. |
| `test_battery.py` | 5 | Tests for the battery-sag model (plant/battery.py) and its wiring into the |
| `test_consistency.py` | 8 | The numbers that live in two places must agree. |
| `test_effectiveness.py` | 12 | The bench-measured thrust/torque surface, and its inverse. |
| `test_gazebo_mapping.py` | 3 | The Gazebo command-side inversion. |
| `test_gazebo_servo.py` | 6 | The Gazebo gimbal servo must be stable at the world's physics step. |
| `test_uncertainty.py` | 8 | Tests for uncertainty propagation (verify/uncertainty.py). |
| | **66** | |
<!-- >>>EMIT:tests -->

- **Allocation round-trip:** over 2000 randomized unsaturated commands the
  realized moment matches the commanded moment to **4.3 × 10⁻¹³ N·m**, frozen in
  `reference/golden/`.
- **Surface inverse:** thrust exact to **< 10⁻⁹ N** across the entire signed
  feasible set; torque exact except within the top and bottom few percent of
  throttle, where up to 0.007 N·m is deliberately traded away to keep thrust
  (§ Results Robustness).
- **Generated artefacts:** `gen_model_sdf.py --check` (the Gazebo composite
  reproduces the YAML to 2 × 10⁻¹⁶ mm of CG error) and `gen_docs.py --check`.

**The suite has earned its keep twice.** It caught an incomplete axis rename — a
three-cycle applied two-thirds of the way — on the commit it was written for. And
it caught the surface inverse losing 0.32 N of thrust within 0.05 N of the
ceiling, which no scenario exercised because no scenario flies at 100% throttle.

**Still missing, and this is what keeps it at 3 rather than 4:**

- **Only one exploratory time-step refinement has been checked.** On the
  nominal lateral case, reducing the control step from 10 ms to 5 ms changed
  settling time by 5 ms and peak outer-ring deflection by 0.013°. At 20 ms,
  delay quantization crossed a saturation boundary and changed settling by
  0.215 s. This supports 10 ms for that case, but it is not a convergence study
  across all scenarios, and `max_step = dt/4` remains an unqualified choice.
- **No matched analytic-versus-Gazebo scenario suite exists.** Both plants and
  both ROS launch paths have run, but only nominal hover behaviour has been
  compared. The five analytic scenarios have not been replayed in Gazebo under
  versioned tolerances, so broad cross-plant agreement is still unproved.
- **Only one torque-free conservation check has been run.** Over 20 s with
  gravity and control disabled, relative kinetic-energy and inertial angular-
  momentum drift were below `8.4e-15` under tight solver tolerances. This checks
  the Newton-Euler/quaternion equations at one state, not conservation across a
  sweep or under the production solver tolerances.

### Validation — "was the right model built?" · level 0

**No validation evidence of any kind exists.** Not a gap in rigour — a gap in
data. Specifically absent:

- **No comparison of simulated vehicle motion against a real flight.** The
  vehicle has not flown.
- **No matched scenario comparison between the analytic and Gazebo models.**
  See above; agreement between two simulators would strengthen verification,
  but would not replace flight validation.
- **No back-check of the fitted surface against held-out bench points.** The fit
  used 119 of 121 measured points and none were reserved.

`tvc.py validate` is named "validate" and performs **verification**. The command
name is historical; `verify/scenarios.py` says so in its first paragraph.

---

## M&S Operations

### Input Pedigree — where the numbers come from · level 3

Full table with per-number provenance: [5-PARAMETERS.md](5-PARAMETERS.md).
Roughly **46% measured, 29% derived, 5% estimated** — but the share is less
informative than which numbers fall where.

**Actuators — measured, with stated error.** A 121-point sweep of both motor
commands (1000–2000 µs, 100 µs steps, 4 s dwell, 50 Hz); the first 1 s of each
dwell discarded for overshoot, and a dwell rejected when the least-squares slope
over its last 3 s exceeded 0.3. 119 points survived.

| model | fit quality |
|---|---|
| thrust surface `f_T` | R² 0.9984, RMSE **0.243 N**, MAE 0.194 N |
| torque surface `f_Q` | R² 0.9965, RMSE **0.0032 N·m**, MAE 0.0025 N·m |
| gimbal inner θ(PWM) | R² 0.9977, RMSE 0.197°, nonlinearity 3.61% of span, hysteresis 0.24°/0.43° |
| gimbal outer θ(PWM) | R² 0.9942, RMSE 0.312°, nonlinearity 6.61% of span, hysteresis 0.35°/0.65° |

Two anchors confirm the coefficients were transcribed correctly, which fit
statistics cannot: `f_T(1,1) = 17.79 N` matches the quoted peak thrust, and
`f_Q(1,−1) = −0.173 N·m` matches the quoted peak reaction torque. Both are
asserted in `tests/test_effectiveness.py`.

Caveats on this data:

- **Fresh-pack fit.** Over 7 × 60 s sustained-load repeats the pack fell
  11.9 → 10.0 V and thrust 13.8 → 12.0 N (−13%). The surface describes the start
  of a flight, not the end. The derating is recorded in the YAML and **not
  modelled**.
- **Hysteresis is measured and not modelled** — up to 0.65° on the outer ring,
  ~9% of that ring's travel.
- The source slides print the two surfaces' R²/RMSE/MAE lines under the wrong
  surfaces; the values above are matched to the correct one.

**Mass properties — weaker, and this is the honest limit of this factor.**
`tools/components.yaml` totals 748 g of point masses of which all but the two
servos are marked `estimated: true`, plus a lumped remainder placed at z = 150 mm
to reach the 1328 g design total. CG and inertia are therefore **design intent,
not measurement**. No mass or balance measurement of the built vehicle has been
recorded.

The consequence is specific and expensive: **the lever arm `L = 0.2111 m` sets
all lateral authority, and it is an unmeasured number.** On top of that,
`geometry.lever_arm_mode` is still marked DECISION DEFERRED — the `rotor_plane`
alternative would cut lateral authority by 21%.

**Cheapest fix available:** balance the assembled vehicle on a knife edge and set
`center_of_mass_override_mm` in `components.yaml`. Hours of work; it would move
Input Pedigree further than anything else on this list.

### Results Uncertainty · level 0

Every uncertainty above is **recorded and never propagated**. Simulation results
are reported as point values with no error bars. Nobody has asked what a 0.243 N
thrust RMSE, a −13% battery derate, or an unmeasured lever arm does to a settling
time or a control margin.

**Cheapest first step:** re-run the five scenarios with `L`, mass and the surface
perturbed by their stated uncertainty, and report the spread of the metrics. The
scenarios are already parameterised and deterministic, so this is a loop, not a
project.

### Results Robustness · level 2

Known divergences between this model and the vehicle, maintained as they are
introduced.

| divergence | effect | status |
|---|---|---|
| **the measured 100 ms motor response is not identified as a delay or a lag** | **decides whether the roll channel is controllable at all.** Same 20° upset: as a first-order lag it recovers cleanly and never saturates; as a pure transport delay it winds up to 72° and saturates 97% of the run. The lateral axes are unaffected either way. | **OPEN — and now shown to be *unanswerable from the existing bench data*.** `tvc-data/motor/identify_dynamics.py` establishes that (a) dead time is structurally unmeasurable on this rig — the command is host-clock-stamped while force is STM32-stamped, so an onset shift is algebraically identical to a dead time; and (b) at the near-hover SNR (~2.4) a synthetic-truth power check recovers a true 44 ms lag as ~0 ms, its p5–p95 interval reaching 0, so a pure delay cannot be rejected. The fix is a **logging change, not a new rig**: the load cell already averages ~20 raw samples into each 20 ms row, so logging at 500 Hz (or using ~6 N steps) resolves it; more repeats do not. |
| motor lag modelled at all | the default is the optimistic reading | modelled; `tvc.py validate` prints the pessimistic number every run |
| battery sag and thrust derate | −13% over a sustained run; the surface is a fresh-pack fit | measured, **not modelled** |
| servo resonance (+11 dB inner, +5 dB outer) | a rate limit cannot represent a lightly damped peak; closed-loop margin near it is unmodelled | deferred |
| gimbal hysteresis | up to 0.65°, ~9% of the outer ring's travel | measured, not modelled |
| no aerodynamics at all | no drag, no ground effect, no wind, no blade flapping | accepted at this stage |
| no sensor model | state feedback is perfect; real noise, bias and latency absent | deferred — **Seam A already exists**, so this drops in without touching the controller |
| **the ROS controller has no state-age or command-timeout failsafe** | a dropped odometry stream stops new control updates, while a transport or actuator may continue applying its last command; a delayed stream can also make the controller act on stale state | **acceptable only for SIL. Add explicit stale-state disarm and actuator-command timeout semantics before HITL or hardware.** |
| the analytic plant has no gimbal-ring or rotor gyroscopic terms and no ground contact | limits analytic↔Gazebo agreement | accepted; keep every scenario airborne |
| Gazebo rotor speed is not a physical RPM | `momentConstant` is a solver scaling and `maxRotVelocity` is solver headroom; the plugin no longer enforces the real 17.79 N ceiling | **accepted deliberately** — it is the price of reproducing the measured surface exactly. The allocator enforces the ceiling instead, and `tests/test_allocation.py` asserts it. |
| **the ROS 2 path holds a 1.5-2.5 deg roll limit cycle the direct path does not** | the two SIL pipelines run identical control code and agree on every channel except the thrust axis; the bridge's transport delay costs phase margin exactly where the vehicle has least inertia and the slowest actuator | **OPEN — measured, bounded, not explained in detail** |
| **above ~17.0 N the feasible tau_P interval excludes zero** | a full-throttle climb applies a forced +0.017 N.m roll torque and the vehicle takes 53 deg of thrust-axis roll before the throttle comes off the stop | real airframe behaviour, modelled correctly; an operational limit rather than a modelling gap |
| Gazebo's gimbal servo settles in 12.5 ms against the 30 ms the plant models | a factor of ~2, not a decade; the servo is nearly but not entirely transparent | bounded by `tests/test_gazebo_servo.py`; a coarser physics step would spend the margin |
| the Gazebo run is reproducible in aggregate but not sample-wise | two consecutive 30 s runs differ by up to 1.5 deg of roll and 12 mm of altitude at any one sample, because the first accepted odometry message can land a physics step apart | why the Gazebo golden compares metrics; was 49 deg before the controller took ownership of the unpause |
| the feasible-set table is binned (200 bins over 18 N) | over-promises roll torque by up to 0.007 N·m below ~8% and above ~95% throttle; the inverse then trades that torque away to keep thrust exact | measured and bounded by test |

**No sensitivity study has been run**, so nothing tells us which of these matters
most. That is what keeps this factor at 2.

**Divergences that were closed**, recorded because closing them changed real
numbers:

- the Gazebo `momentConstant` reproduced only ~50% of the measured roll torque at
  hover and none of its sign asymmetry — fixed by the command-side inversion;
- the rigid body was integrated with a diagonal inertia when `Iyz/Izz = 27.3%` —
  fixed by carrying the full tensor;
- **Gazebo's gimbal servo was numerically unstable at the 1 ms step** and tumbled
  the vehicle from a level, zero-command state — fixed by deriving the gains from
  each ring's inertia;
- **the odometry twist was read as inertial when it is body-frame** (REP-105).
  At the world's 12.2° spawn that is 21% of the descent rate appearing as
  lateral velocity that is not there. It was wrong in all three consumers, and
  `simulator_node` published the wrong frame as well — so the analytic plant and
  gz-sim disagreed about what the same message meant, which is the one thing two
  plants sharing one controller may not do;
- **the first two odometry messages carry a garbage twist** — gz-sim's
  `OdometryPublisher` differences the pose against a zero-initialised previous
  pose, so message one reports 651 m/s and 58 rad/s. The controller's rate loop
  saw that and saturated the gimbal on step one. They are now rejected by
  checking the reported twist against the pose derivative, which needs no tuned
  threshold: the position has not moved at all while the twist claims hundreds
  of m/s;
- **`tvc.py view3d` and the GUI's 3D button raised `AttributeError` on the first
  frame** — `quat_to_rotmat` is flight code and returns a tuple of rows, and the
  viewer asked it for `.T`. Neither had opened since the math moved into `gnc/`.

---

## Supporting Evidence

### Use History · level 2

- **The Gazebo hover, re-flown under the shared flight code**: spawned tilted,
  level in 1.03 s, 2.000 m held with no drift, frozen as
  `reference/golden/hover_baseline.json`. The old demo that produced the
  original result carried its own duplicate controller. The old
  behaviour is therefore use history for code that no longer exists.
- The analytic simulator drives the GUI and the 3D viewer, and is exercised on
  every commit by CI.
- **The Gazebo hover flies, and it is now the second frozen baseline.** From the
  world's deliberate 10°/−7° spawn the vehicle recovers level in **1.03 s**,
  holds 2.000 m with 1 × 10⁻⁸ m of drift, and settles the gimbal at zero with
  thrust at exactly *mg* = 13.03 N. `reference/golden/hover_baseline.json`
  freezes fourteen metrics of that run and `tvc.py hover --check-golden`
  compares against it. The picture is
  [hover_baseline.png](../reference/golden/hover_baseline.png).

  It did not fly a week ago; it tumbled to 180° in about a second, and the cause
  was **not** in the flight code. Gazebo's `JointPositionController` had
  hand-written gains of `p=60, d=1.0` against a gimbal ring of 8.5 × 10⁻⁵ kg·m²
  at a 1 ms step, so the explicit damping update `ω ← ω(1 − d·dt/I)` ran at
  `d·dt/I = 11.8` and diverged. The joint chattered against its ±5 N·m clamp
  and the reaction tumbled the airframe. It reproduced with the **controller not
  running at all** and the gimbal commanded to exactly zero, which is what made
  it attributable. The gains are now derived per ring from that ring's own
  inertia, and `tests/test_gazebo_servo.py` asserts the stability condition
  against the world files' actual step.

- **The ROS 2 stack runs, and the two SIL paths disagree in exactly one
  channel.** `ros2 launch tvc_control gazebo.launch.py` brings up all five
  processes, messages cross the `ros_gz_bridge` in both directions, and the
  vehicle holds 2.000 m with pitch and yaw inside ±0.01°. But the thrust-axis
  channel sits in a persistent **1.5–2.5° limit cycle** that the direct
  gz-transport path does not have (it settles to 1 × 10⁻⁴°). One channel differs
  and it is the one with 11.5× less inertia and a ~3× slower actuator — the one
  where the bridge's added transport delay actually costs phase margin. Bounded,
  attributable, and unresolved.

  Getting there needed two fixes, both of which had survived a clean
  `colcon build`: the YAML source of truth was installed to `share/` where
  nothing reads it, so every node died on startup; and the retired-parameter
  guard was written as `declare_parameter(name, Parameter.Type.NOT_SET)`, which
  rclpy on Jazzy rejects outright, so the guard killed every node whether a
  retired parameter was set or not. **A package that builds is not a package
  that runs**, and only `ros2 launch` distinguishes them.

- **Above ~17.0 N of thrust the vehicle cannot command zero roll torque.** The
  feasible τ_P interval stops containing zero — at 17.4 N it is
  `[+0.0090, +0.0301] N·m` — because both props are near their stops and the
  coax asymmetry no longer cancels. The allocator clamps to the nearest
  reachable value, so a full-throttle climb applies a forced positive roll
  torque on a 0.00196 kg·m² axis: **53° of thrust-axis roll** before the
  throttle comes off the stop, recovered by 3 s. This is real airframe
  behaviour, correctly modelled, and it has an operational consequence — the
  vehicle cannot hold roll attitude during a maximum-rate climb.

  `roll_headroom()` used to report `min(|lo|,|hi|)` there, which reads as
  authority in both directions where there is none in either. It returns zero
  now. Nothing had noticed because `validate` printed only final values; the
  scenario plots are what made it visible.

- **Gazebo's gimbal servo is fast, but not by much.** It is squeezed from below
  by the 30 ms transport delay the *plant* models — Gazebo must not add a second
  lag on top — and from above by `2ζωₙ·dt < 1` at the 1 ms step. Those meet near
  ωₙ = 400 rad/s: 12.5 ms of settling against 30 ms, and `d·dt/I = 0.64`. There
  is roughly a factor of two of room. A coarser physics step would spend it.

- No result from this simulator has yet been used for a design decision that was
  subsequently checked against hardware.

### M&S Management · level 4

- `vehicle_params.yaml` is a single source of truth with **no fallback anywhere**;
  a missing file raises rather than defaulting.
- Three artefacts are generated from it and **all three are `--check`ed in CI**:
  the Gazebo model, the numeric documentation sections, and the frozen numerical
  baseline. A generated file that has been hand-edited fails.
- Mass properties are regenerated from `components.yaml` through a
  sentinel-delimited splice that preserves the hand-written comments around them.
- Conventions are centralized in [4-CONVENTIONS.md](4-CONVENTIONS.md) and
  asserted by test, not merely documented.
- Under version control with a tagged restore point and a frozen baseline.
- The historical weakness — stale constants duplicated across files, the 20 N
  thrust figure surviving in three places after being superseded — is what the
  single-source-of-truth work removed, and `test_consistency.py` is what keeps it
  removed.

### People Qualifications — not assessed

Undergraduate research project. Recorded as not assessed rather than scored; what
carries credibility here is the evidence in the rows above.

---

## How to use this file

When simulation results inform a decision — a gain choice, a structural change, a
claim in a report — state alongside them:

1. **which factors are level 0** (right now: Validation and Results Uncertainty);
2. **the headline caveat**;
3. **the specific divergences from Results Robustness that could affect *that*
   decision.**

Update this file in the same commit as any change that moves a level, and say
which factor moved and why.

### The three cheapest things that would raise a level

| do this | cost | moves |
|---|---|---|
| re-log the motor step response at 500 Hz (not a new rig — the rig already samples ~1 kHz and discards it) to distinguish delay from lag | one session | Results Robustness, and possibly the roll gains |
| knife-edge CG measurement of the assembled vehicle | hours | Input Pedigree |
| perturb `L`, mass and the surface by their stated uncertainty; re-run the scenarios | a loop over existing code | Results Uncertainty 0 → 2 |
