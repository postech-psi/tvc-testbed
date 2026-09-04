# Simulation credibility record

How much this simulator's output should be trusted, and on what evidence.

Structured after **NASA-STD-7009**, *Standard for Models and Simulations*, which
assesses credibility on eight factors in three categories and requires that
results reaching a decision-maker carry explicit caveats, an uncertainty
estimate, and the assessment itself — rather than arriving as bare numbers.

> **Scope of the borrowing.** The eight factor names and the three categories
> are the standard's. The 0–4 level *anchors* below are project-local: they are
> written for this vehicle and this team, not lifted from the standard's own
> level definitions. Treat the numbers as a shared vocabulary for "how much
> evidence is behind this", not as a NASA-conformant assessment.
>
> **Blank is a real answer.** A factor with no evidence is recorded as 0 and
> left visibly empty. The point of this file is to convert unexamined confidence
> into stated ignorance, so filling a cell to make the table look finished
> defeats it.

Levels: **0** no evidence · **1** informal/anecdotal · **2** documented but
unverified · **3** verified against an independent reference · **4** verified
plus quantified uncertainty.

Last updated: end of the flight-software restructure (Phases 0-7; Phase 1, the
first colcon build, is still outstanding and needs the devcontainer).

---

## Summary

| category | factor | level | one-line basis |
|---|---|---|---|
| M&S Development | Verification | **3** | 16-test suite + 5 scenarios + frozen baseline, all gated in CI |
| M&S Development | Validation | **0** | *nothing has been compared against the real vehicle* |
| M&S Operations | Input Pedigree | **3** | actuators bench-measured with stated fit error; mass properties are CAD + estimates |
| M&S Operations | Results Uncertainty | **0** | measurement uncertainty is recorded but never propagated |
| M&S Operations | Results Robustness | **2** | divergences enumerated and one quantified (motor lag); sensitivity study still untested |
| Supporting Evidence | Use History | **1** | one Gazebo hover demo; the ROS2 path has never run |
| Supporting Evidence | M&S Management | **3** | single source of truth, generated model, version control, restore point |
| Supporting Evidence | People Qualifications | — | not assessed |

**Headline caveat, to be repeated wherever results are shown:** this simulator
has never been compared against flight data. It reproduces bench-measured
*actuator* behaviour; it has not been shown to reproduce *vehicle* behaviour.

---

## M&S Development

### Verification — "was the model built right?" · level 2

Have:
- `sim/validate_control.py` — four scenarios (lateral upset, roll upset, climb,
  tilt feedforward A/B), each with a threshold chosen to catch a specific
  structural mistake rather than to grade performance.
- Allocation round-trip: over 2000 randomized unsaturated commands the realized
  moment matches the commanded moment to **4.3e−13 N·m**, frozen in
  `sim/golden/analytic_baseline.json`.
- Surface inverse: thrust error < 1e−9 N, torque error < 1e−11 N·m across the
  signed feasible set.
- `tools/gen_model_sdf.py --check` — the Gazebo model reproduces the YAML's mass
  properties (CG error 2e−16 mm).

Since raised to 3 by: `tests/` (16 tests) wired into a blocking CI job. It
enforces the flight code's porting discipline by parsing it (no numpy/scipy/
yaml/ROS import, no file I/O, no unbounded loop, no reach into the plant, and
numpy blocked at the import hook), asserts the axis convention against the
generated SDF including that every gimbal joint is actually driven by a plugin,
checks the tau_P sign chain end to end, and verifies the Gazebo inversion
round-trips over the measured envelope. The frozen baseline and the
SDF-vs-parameters check run there too.

The suite earned its keep immediately: it caught an incomplete axis rename (a
three-cycle applied two-thirds of the way) on the commit it was written for.

Still missing:
- Integrator convergence never checked (no step-size refinement study).
- No cross-plant comparison has been RUN. The machinery for it now exists --
  both plants share one controller, one gain file and one actuator chain, and
  the two launch files differ only in which plant process starts -- but the
  first colcon build has not happened, so analytic-vs-Gazebo agreement remains
  a claim about the architecture rather than a measurement.
- No conservation check (energy/momentum with control off).

Raise to 3 by: `tests/` + CI (restructure Phase 6/7), a step-refinement study,
and the sign-chain assertion.

### Validation — "was the right model built?" · level 0

**No validation evidence of any kind exists.** Not a gap in rigour — a gap in
data. Specifically absent:

- No comparison of simulated vehicle motion against a real flight. The vehicle
  has not flown.
- No cross-validation between the two plants (analytic vs Gazebo). They
  currently run *different controllers with different gains*, so the comparison
  is not yet meaningful; restructure Phase 5 makes it possible.
- No back-check of the fitted surface against held-out bench points. The fit
  used 119 of 121 measured points and none were reserved.

Note that `sim/validate_control.py` is named "validate" but performs
**verification** — it checks internal consistency, not agreement with reality.
The name is misleading and is on the list to change.

---

## M&S Operations

### Input Pedigree — where the numbers come from · level 3

**Actuators — measured, with stated error.** 121-point sweep of both motor
commands (1000–2000 µs, 100 µs steps, 4 s dwell, 50 Hz); first 1 s of each dwell
discarded for overshoot and a dwell rejected when the least-squares slope over
its last 3 s exceeded 0.3 — 119 points survived.

| model | fit quality |
|---|---|
| thrust surface `f_T(u_A, u_B)` | R² 0.9984, RMSE **0.243 N**, MAE 0.194 N |
| torque surface `f_Q(u_A, u_B)` | R² 0.9965, RMSE **0.0032 N·m**, MAE 0.0025 N·m |
| gimbal inner θ(PWM) | R² 0.9977, RMSE 0.197°, nonlinearity 3.61 % of span, hysteresis 0.24°/0.43° |
| gimbal outer θ(PWM) | R² 0.9942, RMSE 0.312°, nonlinearity 6.61 % of span, hysteresis 0.35°/0.65° |

Anchors that confirm the coefficients: `f_T(1,1) = 17.79 N` matches the quoted
peak thrust; `f_Q(1,−1) = −0.173 N·m` matches the quoted peak reaction torque.

Caveats on this data:
- **Fresh-pack fit.** Over 7×60 s sustained-load repeats the pack fell 11.9 →
  10.0 V and thrust 13.8 → 12.0 N (−13 %). The surface describes the start of a
  flight, not the end. Derating is recorded in the YAML but **not modelled**.
- **Hysteresis is measured and not modelled** (up to 0.65° on the outer ring,
  ~9 % of that ring's travel).
- The source slides print the two surfaces' R²/RMSE/MAE lines under the wrong
  surfaces; the values above are matched to the correct one.

**Mass properties — weaker, and this is the honest limit of this factor.**
`tools/components.yaml` totals 748 g of point masses of which all but the two
servos are marked `estimated: true`, plus a 580 g lumped remainder placed at
z = 150 mm to reach the 1328 g design total. CG and inertia are therefore
*design intent*, not measurement. No mass or balance measurement of the built
vehicle has been recorded.

Consequence: the lever arm L = 0.2111 m sets all lateral authority, and it is an
unmeasured number. `geometry.lever_arm_mode` is still marked DECISION DEFERRED —
the `rotor_plane` alternative would cut lateral authority 21 %.

### Results Uncertainty · level 0

Every uncertainty above is **recorded but never propagated**. Simulation results
are reported as point values with no error bars. Nobody has asked what a 0.243 N
thrust RMSE, a −13 % battery derate, or an unmeasured lever arm does to a
settling time or a control margin.

Cheapest first step: re-run the four scenarios with L, mass and the surface
perturbed by their stated uncertainty, and report the spread of the metrics.

### Results Robustness · level 2

Known model divergences are enumerated and will be maintained here as the
restructure introduces them:

| divergence | effect | status |
|---|---|---|
| Gazebo `momentConstant` is linear and symmetric | reproduces only ~50 % of measured axial torque at hover, and none of its sign asymmetry | fix scheduled (command-side inversion) |
| **the 100 ms motor response is not identified as a delay or a lag** | **decides whether the roll channel is controllable at all.** Measured, both readings, same 20 deg upset: as a first-order lag the channel recovers cleanly and never saturates; as a pure transport delay it winds up to ~70 deg and saturates 94% of the run. The lateral axes are unaffected either way. | **OPEN — one bench run resolves it. Highest-value measurement outstanding.** |
| motor lag now modelled (`plant/actuators.py::MotorLag`) | default is the optimistic reading | modelled; `sim/validate_control.py` prints the pessimistic number every run |
| rigid-body inertia treated as diagonal | `Iyz/Izz = 27.3 %`, `Ixz/Izz = 15.4 %` — dominates the roll-axis residual | fix scheduled |
| servo resonance (+11 dB inner, +5 dB outer) | a rate limit cannot represent it; closed-loop margin near that peak is unmodelled | deferred |
| gimbal hysteresis | unmodelled | deferred |
| no sensor model | state feedback is perfect; real noise/delay absent | deferred (seam exists) |
| analytic plant has no gimbal-ring or rotor gyroscopic terms, no ground contact | limits analytic↔Gazebo agreement | accepted; keep scenarios airborne |

No sensitivity study has been run — nothing tells us which of these matters
most. That is what keeps this at 2.

---

## Supporting Evidence

### Use History · level 1

- One Gazebo hover demo (`sim/run_hover.sh`), used repeatedly during tuning;
  pass thresholds are altitude error < 0.3 m, tilt < 15°, drift < 1.0 m.
- The analytic simulator drives the Tkinter GUI and the 3D viewer.
- **The ROS2 path has never been built or run** — `build/`/`install/` contain
  only the unrelated `tvc_demo` package.
- No results from this simulator have yet been used for a design decision that
  was subsequently checked against hardware.

### M&S Management · level 3

- `sim/vehicle_params.yaml` is a single source of truth; the Gazebo model is
  generated from it by `tools/gen_model_sdf.py` and cannot silently drift.
- Mass properties are regenerated from `tools/components.yaml` through a
  sentinel-delimited splice that preserves hand-written comments.
- Under version control with a tagged restore point (`pre-unify`) and a frozen
  numerical baseline (`sim/golden/`).
- Conventions are centralized in [CONVENTIONS.md](CONVENTIONS.md).
- Weak points: no CI; several stale constants have historically been duplicated
  across files (the 20 N thrust figure survived in three places after being
  superseded), which is what the single-source-of-truth work is fixing.

### People Qualifications — not assessed

Undergraduate research project. Recorded as not assessed rather than scored;
what carries the credibility here is the evidence in the rows above.

---

## How to use this file

When simulation results inform a decision — a gain choice, a structural change,
a claim in a report — state alongside them: which factors are level 0, the
headline caveat, and the specific divergences from Results Robustness that could
affect *that* decision. Update this file in the same commit as any change that
moves a level, and say which factor moved and why.
