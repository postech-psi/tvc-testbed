# Simulator fidelity upgrades — design

**Date:** 2026-09-10
**Status:** approved for spec review
**Scope:** four native modules this round (uncertainty propagation, motor
delay-vs-lag identifiability, battery sag, aerodynamics) plus a cross-plant
validation suite as the immediate sequel.

## Goal

Move the simulator's credibility (`docs/7-CREDIBILITY.md`) off two level-0
factors and reduce two unmodelled divergences, using **existing bench data only**
(no new test runs are possible). Every new fidelity effect is a toggle that
defaults **off**, so the frozen goldens (`reference/golden/`) and the
deterministic MIL path do not move unless explicitly asked.

## Design principles (inherited from the repo)

- **`tvc-data` analyses bench measurements; `tvc-testbed` models the vehicle.**
  System-ID scripts live in `tvc-data`; only resulting numbers cross into
  `vehicle_params.yaml`, and only when the data actually supports them.
- **Seams already exist** — the plant integrates *achieved* actuator states
  (`harness/mil.py`) and the controller consumes an `EstimatedState`
  (`plant/sensors.py`). New effects attach at existing seams, not by rewriting
  loops.
- **Every number carries provenance.** Estimated parameters are tagged
  `estimated` and conclusions resting on them are flagged provisional.
- **Assert by test, not by comment.** Each module ships unit tests; toggled-off
  behaviour is asserted bit-identical to today where practical.

---

## Module 1 — Uncertainty propagation (Results Uncertainty 0 → 2)

**Where:** `src/tvc_control/tvc_control/verify/uncertainty.py`, exposed as
`python tvc.py validate --uncertainty [--samples N] [--seed S]`.

**What it does.** Wraps the existing 5 scenarios (`verify/scenarios.py`) in a
seeded Monte-Carlo loop. For each sample it perturbs the documented-uncertain
inputs, re-runs the scenario deterministically, and collects the scalar metrics
already produced by `_compute_metrics`. Reports per-metric **mean, std, p5, p95,
min, max** per scenario.

**Perturbed inputs and their stated uncertainties** (sourced from
`docs/5-PARAMETERS.md` / `docs/7-CREDIBILITY.md` at implementation time; a single
`UncertaintySpec` dataclass holds the distributions so they are auditable in one
place):

| input | distribution | basis |
|---|---|---|
| lever arm `L` | ± range covering the `pivot`↔`rotor_plane` 21% span | unmeasured; DECISION DEFERRED |
| mass `m` and inertia tensor | ± % (design-intent, not weighed) | CAD provenance |
| thrust surface | additive Gaussian, σ = fit RMSE (≈0.243 N) | measured fit error |
| torque surface | additive Gaussian, σ = fit RMSE (≈0.0032 N·m) | measured fit error |

**Interface.** `run_uncertainty(scenario, n_samples, seed) -> MetricSpread`.
Pure function of (scenario, N, seed) → reproducible. No effect on the default
`validate` path.

**Testing.** (a) `--samples 1 --seed` with zero perturbation reproduces the
deterministic scenario metrics exactly; (b) spread is monotonic in perturbation
magnitude for a known-linear metric; (c) fixed seed → identical spread across
runs.

---

## Module 2 — Motor delay-vs-lag identifiability (honest null result)

**Where:** `tvc-data/motor/identify_dynamics.py` + a short generated report
under `tvc-data/motor/out/`. **No change to `vehicle_params.yaml`.**

**Finding to reproduce (from the pasted analysis, cross-checked):** at the
near-hover operating point the time constant τ is **not identifiable** from this
data. The command is host-clock-stamped in bursts (≈0.7% drift) while force is
STM32-stamped, so an unknown onset shift is algebraically identical to dead time;
and at SNR ≈ 2.2 a synthetic-truth power check recovers an injected 44 ms lag as
0 ms every trial.

**What the script produces.**
1. **Step-edge extraction** from `loadcell.csv` aligned to command transitions.
2. **Variable-projection fit with a free onset** (baseline + amplitude profiled
   out; scan the shared τ; τ→0 = pure-delay hypothesis).
3. **Synthetic-truth power check**: real step windows, real noise, real onset
   scatter, known injected τ → demonstrate non-recovery near hover.
4. **Resolvable-regime check**: large steps from rest (SNR ≈ 22) give ≈268 ms
   onset + τ ≈ 96 ms, but that is ESC start-up + spin-up (`T ∝ ω²`), not the
   operating point — reported as such, not used for control design.
5. **Recommendation**: log at 500 Hz (the rig already averages ≈20 raw samples
   per row — `force_count`), or use ~6 N steps. Not "more repeats".

**Outcome for the model:** the `motor_dynamics` switch stays; default stays
`first_order`; the 65°/123° phase table in the docs stays unresolved. The
credibility doc's "highest-value measurement outstanding" row is updated to note
it is answerable by a logging change, not a new rig.

**Testing.** The synthetic-truth power check is itself the test: assert that a
known injected τ is *not* recovered under the near-hover noise/onset model
(guards against a future "fix" that silently claims identifiability).

---

## Module 3 — Battery sag (measured, then modelled)

**Where:** identification in `tvc-data/motor/identify_battery.py`; model in
`src/tvc_control/tvc_control/plant/battery.py`. Toggle in `SimConfig`
(`battery_sag: bool = False`).

**Identification (from the 7 back-to-back `A1850_B1850` sustained runs).**
- thrust-vs-voltage sensitivity (N/V) at fixed command, from `loadcell.csv` +
  `battery.csv`;
- a voltage-vs-drawn-charge law via **coulomb counting** (integrate
  `current_a` → mAh), the most physical option and the one the measured current
  channel supports.
Emits identified constants to a small YAML/JSON the model reads (kept in
`tvc-data/motor/out/`, mirrored into `vehicle_params.yaml` under
`motor_dynamics.sustained_load` with `measured` provenance).

**Model.** `BatteryState` integrates commanded current over the run → SoC →
voltage; the thrust surface is derated by `thrust_sensitivity_n_per_v ×
(V_nominal − V(SoC))`. Wired at the same seam as `MotorLag` in
`ActuatorChain.update` so the rigid body sees derated achieved thrust.

**Testing.** (a) toggle off ⇒ thrust identical to today; (b) over a sustained
hover the modelled thrust falls in family with the measured 13.8→12.0 N trend;
(c) coulomb integral is conserved / monotonic.

---

## Module 4 — Aerodynamics (parametric, explicitly estimated)

**Where:** `src/tvc_control/tvc_control/plant/aero.py`; force added inside
`plant/rigidbody.dynamics`. Toggle via a `params.aero_enabled` flag (default
off); parameters live in `vehicle_params.yaml` under an `aero:` block, every one
tagged `estimated`.

**Model.** Translational quadratic drag
`F_drag = -0.5 ρ Cd A |v| v` (inertial velocity), with an **optional
ground-effect** thrust multiplier as a function of altitude/rotor-radius. `Cd`,
reference area `A`, and `ρ` are estimates derived from geometry; ground-effect
form is the standard `T/T_∞ = 1/(1 - (R/4z)²)` clamped near the ground.
No rotational aero damping this round (documented as deferred).

**Rationale for parametric-only.** There is *no* aero bench data, so this is
design-intent, not metrology. It exists so scenarios can ask "does a plausible
drag change any conclusion?" — an input to robustness, flagged provisional.

**Testing.** (a) toggle off ⇒ dynamics bit-identical to today; (b) drag opposes
velocity and is zero at rest; (c) ground effect → 1 at high altitude and > 1 near
ground; (d) energy check: with drag on and control off, kinetic energy is
non-increasing.

---

## Module 5 — Cross-plant validation suite (sequel; Gazebo runs in the user's env)

**Where:** `src/tvc_control/tvc_control/verify/cross_plant.py`.

**What it does.** Replays the same initial states + setpoints for the 5
scenarios on both the analytic plant and Gazebo, compares attitude peak/RMS,
settling time, peak |τ_P|, altitude, and final position under **versioned
tolerances** ratified from repeated runs. Tolerances must cover the known Gazebo
gyroscopic/contact/timing effects without hiding a sign, frame, or allocation
error.

**Dependency.** Requires Gazebo execution, which the user confirmed runs in their
environment. Implemented after modules 1–4; the harness + analytic side +
tolerance framework are built and unit-tested first, then Gazebo numbers wired
in. This is also the path most likely to explain the ROS 2 thrust-axis limit
cycle.

---

## Rollout / commits

Incremental, one module per commit, each with tests green:
1. uncertainty harness (+ tests)
2. delay-vs-lag identifiability script (+ report, + power-check test)
3. battery identification (+ model + tests + YAML update)
4. aero model (+ tests)
5. cross-plant suite (after Gazebo verification)

Credibility doc updated in the same commit as any factor/divergence it moves.
