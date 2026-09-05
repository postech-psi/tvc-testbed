# 6 — Credibility

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
| Supporting Evidence | Use History | **1** | one Gazebo hover demo; the ROS 2 path has never run |
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
| `test_allocation.py` | 6 | Control allocation: does it realize the moment it was asked for? |
| `test_attitude_error.py` | 5 | The quaternion attitude error, and why it replaced the Euler difference. |
| `test_axis_convention.py` | 11 | The axis convention, asserted rather than documented. |
| `test_consistency.py` | 13 | The numbers that live in two places must agree. |
| `test_container.py` | 7 | The Dockerfile must build, and it must not be a second copy of requirements.txt. |
| `test_effectiveness.py` | 12 | The bench-measured thrust/torque surface, and its inverse. |
| `test_gazebo_mapping.py` | 3 | The Gazebo command-side inversion. |
| `test_gnc_purity.py` | 6 | Enforce the porting discipline on the flight code. |
| `test_scenarios.py` | 2 | The five closed-loop scenarios, run as tests. |
| | **65** | |
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

- **Integrator convergence has never been checked.** No step-size refinement
  study exists. `max_step = dt/4` is a plausible choice, not a justified one.
- **No cross-plant comparison has been RUN.** The machinery now exists — both
  plants share one controller, one gain file and one actuator chain, and the two
  launch files differ only in which plant process starts — but the first colcon
  build has not happened, so analytic-versus-Gazebo agreement remains **a claim
  about the architecture rather than a measurement**.
- **No conservation check** (energy and momentum with control off).

### Validation — "was the right model built?" · level 0

**No validation evidence of any kind exists.** Not a gap in rigour — a gap in
data. Specifically absent:

- **No comparison of simulated vehicle motion against a real flight.** The
  vehicle has not flown.
- **No cross-validation between the two plants.** See above.
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
| **the measured 100 ms motor response is not identified as a delay or a lag** | **decides whether the roll channel is controllable at all.** Same 20° upset: as a first-order lag it recovers cleanly and never saturates; as a pure transport delay it winds up to 72° and saturates 97% of the run. The lateral axes are unaffected either way. | **OPEN — one bench run resolves it. The highest-value measurement outstanding.** |
| motor lag modelled at all | the default is the optimistic reading | modelled; `tvc.py validate` prints the pessimistic number every run |
| battery sag and thrust derate | −13% over a sustained run; the surface is a fresh-pack fit | measured, **not modelled** |
| servo resonance (+11 dB inner, +5 dB outer) | a rate limit cannot represent a lightly damped peak; closed-loop margin near it is unmodelled | deferred |
| gimbal hysteresis | up to 0.65°, ~9% of the outer ring's travel | measured, not modelled |
| no aerodynamics at all | no drag, no ground effect, no wind, no blade flapping | accepted at this stage |
| no sensor model | state feedback is perfect; real noise, bias and latency absent | deferred — **Seam A already exists**, so this drops in without touching the controller |
| the analytic plant has no gimbal-ring or rotor gyroscopic terms and no ground contact | limits analytic↔Gazebo agreement | accepted; keep every scenario airborne |
| Gazebo rotor speed is not a physical RPM | `momentConstant` is a solver scaling and `maxRotVelocity` is solver headroom; the plugin no longer enforces the real 17.79 N ceiling | **accepted deliberately** — it is the price of reproducing the measured surface exactly. The allocator enforces the ceiling instead, and `tests/test_allocation.py` asserts it. |
| the feasible-set table is binned (200 bins over 18 N) | over-promises roll torque by up to 0.007 N·m below ~8% and above ~95% throttle; the inverse then trades that torque away to keep thrust exact | measured and bounded by test |

**No sensitivity study has been run**, so nothing tells us which of these matters
most. That is what keeps this factor at 2.

**Two divergences that were closed** and are recorded because closing them
changed real numbers: the Gazebo `momentConstant` used to reproduce only ~50% of
the measured roll torque at hover and none of its sign asymmetry (fixed by the
command-side inversion), and the rigid body used to be integrated with a diagonal
inertia when `Iyz/Izz = 27.3%` (fixed by carrying the full tensor).

---

## Supporting Evidence

### Use History · level 1

- **One Gazebo hover demo**, used repeatedly during tuning: spawned tilted,
  recovered level in ~2 s, held 2.00 m with no drift. That demo carried its own
  duplicate controller. The rewritten harness calls the shared flight code
  instead, and does *not* reproduce the result — see the next bullet. The old
  behaviour is therefore use history for code that no longer exists.
- The analytic simulator drives the GUI and the 3D viewer, and is exercised on
  every commit by CI.
- **The ROS 2 path builds but has never been run.** `colcon build` now succeeds
  for both packages in the devcontainer, all three node modules import from the
  installed package, and `gazebo.launch.py` produces a valid launch description.
  No ROS 2 message has yet crossed the `ros_gz_bridge`, so the type strings
  remain unverified.
- **The Gazebo hover has been re-flown, and it does not recover.** First run of
  the rewritten `harness/gz.py`: from the world's deliberate 10°/−7° spawn the
  vehicle diverges to 180° tilt in ~1.1 s with the gimbal pinned at 6.9° of 7.0
  for the whole run, landing at 0.25 m with 1.42 m of drift. The analytic plant
  recovers the comparable upset with 1% saturation. **That is a MIL/SIL
  disagreement, not a tuning problem** — a free-flying vehicle feels no gravity
  moment about its CG, so nothing tips it but its own control action. It is the
  single most informative open result in this document, and it is exactly what
  cross-plant validation exists to find. The SDF servos are already neutralised,
  so the next suspect is the sign of the actuator chain between
  `ActuatorSetpoint` and the Gazebo joint and rotor topics.
- **There is still no Gazebo golden**, and there should not be one until the
  above is understood — freezing a divergence as a reference makes it permanent.
  See [reference/golden/README.md](../reference/golden/README.md).
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
| one bench run distinguishing motor delay from lag | one session | Results Robustness, and possibly the roll gains |
| knife-edge CG measurement of the assembled vehicle | hours | Input Pedigree |
| perturb `L`, mass and the surface by their stated uncertainty; re-run the scenarios | a loop over existing code | Results Uncertainty 0 → 2 |
