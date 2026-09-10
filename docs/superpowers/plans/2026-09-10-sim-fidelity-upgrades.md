# Simulator Fidelity Upgrades Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add uncertainty propagation, an honest motor delay-vs-lag identifiability study, a data-derived battery-sag model, and a parametric aero model to the TVC simulator — every fidelity effect toggleable and defaulting off — then a cross-plant validation suite.

**Architecture:** New effects attach at existing seams (`ActuatorChain`, `rigidbody.dynamics`, the `SimConfig`/scenario runner). Data-analysis scripts live in `tvc-data`; only supported numbers cross into `vehicle_params.yaml`. Toggles default off so `reference/golden/` and the MIL path stay bit-identical unless asked.

**Tech Stack:** Python 3, numpy, scipy, pandas, pytest, PyYAML. Repo test command: `python -m pytest -q` from `tvc-testbed`.

**Spec:** `docs/superpowers/specs/2026-09-10-sim-fidelity-upgrades-design.md`

## Global Constraints

- Every new fidelity effect defaults **off**; toggled-off behaviour must be bit-identical to current output (assert in tests where practical).
- `gnc/` stays pure (no scipy/numpy/yaml/clock). New numeric code goes in `plant/`, `verify/`, `harness/`, `apps/`, or `tvc-data/`.
- No literal physical fallbacks; numbers come from `vehicle_params.yaml` via `config.load*`.
- Data analysis of bench measurements lives in `tvc-data`; simulation output never written into `tvc-data`.
- Estimated parameters tagged `estimated` in YAML/report; conclusions resting on them flagged provisional.
- Commit per task on `main` with the repo's Co-Authored-By trailer.

---

## Module 1 — Uncertainty propagation

### Task 1: `UncertaintySpec` + perturbed `VehicleParams` factory

**Files:**
- Create: `src/tvc_control/tvc_control/verify/uncertainty.py`
- Test: `tests/test_uncertainty.py`

**Interfaces:**
- Consumes: `VehicleParams` (gnc/params.py), `ThrustTorqueSurface` (gnc/effectiveness.py), `load_vehicle_params` (config.py).
- Produces:
  - `UncertaintySpec` dataclass: `L_rel_range: tuple`, `mass_rel_sigma: float`, `inertia_rel_sigma: float`, `thrust_surface_sigma_n: float`, `torque_surface_sigma_nm: float`, with a `default()` classmethod sourcing σ from the YAML fit block (thrust RMSE 0.243 N, torque RMSE 0.0032 N·m) and L range spanning pivot↔rotor_plane.
  - `perturb_params(vp, spec, rng) -> VehicleParams`: returns a copy with L, m, Ix/Iy/Iz (+products scaled with their axis), and a surface wrapped to add per-call constant offsets drawn from σ. `rng=None` or zero σ returns an equivalent-behaviour vehicle.

- [ ] Step 1: Write failing tests — (a) `perturb_params(vp, UncertaintySpec.zero(), rng)` yields identical scenario metrics to the unperturbed run; (b) with nonzero `mass_rel_sigma` and a fixed seed, `m` shifts by the expected z-score; (c) surface offset is applied additively to thrust.
- [ ] Step 2: Run tests, verify they fail (module missing).
- [ ] Step 3: Implement `UncertaintySpec`, `.default()`, `.zero()`, and `perturb_params`. Surface perturbation: subclass/wrap `ThrustTorqueSurface` so `thrust()` returns base + offset_n, `torque()` returns base + offset_nm, offsets fixed at construction from `rng`.
- [ ] Step 4: Run tests, verify pass.
- [ ] Step 5: Commit.

### Task 2: Monte-Carlo runner + metric spread

**Files:**
- Modify: `src/tvc_control/tvc_control/verify/uncertainty.py`
- Test: `tests/test_uncertainty.py`

**Interfaces:**
- Produces:
  - `run_uncertainty(scenario_fn, vp, gains, spec, n_samples, seed) -> dict[str, dict]`: maps each scalar metric name to `{mean, std, p5, p95, min, max, n}`. Reuses the existing `scenario_fn(vp, gains, verbose)` signature from `verify/scenarios.py`, substituting a perturbed `vp` per sample.
  - `format_spread(name, spread) -> str` for terminal output.

- [ ] Step 1: Write failing tests — (a) `n_samples=1, seed` with `UncertaintySpec.zero()` reproduces the deterministic metric as mean with std 0; (b) fixed seed → identical spread dict across two calls; (c) larger `mass_rel_sigma` → larger std on `settling_time_s` for the lateral scenario (monotonicity).
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement `run_uncertainty` (seeded `np.random.default_rng`, loop, collect metrics, aggregate with `np.percentile`) and `format_spread`.
- [ ] Step 4: Run, verify pass.
- [ ] Step 5: Commit.

### Task 3: `--uncertainty` CLI wiring

**Files:**
- Modify: `src/tvc_control/tvc_control/verify/scenarios.py` (add `--uncertainty`, `--samples`, `--seed`; default off)
- Test: `tests/test_uncertainty.py`

**Interfaces:**
- Consumes: `run_uncertainty`, `format_spread`.

- [ ] Step 1: Write failing test — calling `scenarios.main(["--uncertainty","--samples","8","--seed","0"])` returns 0 and does not raise; without the flag, `main([])` behaviour/exit code is unchanged.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Add the args; when `--uncertainty`, after the normal deterministic line for each scenario, print the spread of key metrics. Deterministic path untouched when flag absent.
- [ ] Step 4: Run full suite `python -m pytest -q` (goldens must still pass), verify pass.
- [ ] Step 5: Commit.

---

## Module 2 — Motor delay-vs-lag identifiability (tvc-data)

### Task 4: Step-edge extraction + variable-projection fit + power check

**Files:**
- Create: `C:/Users/tae06/CODE/tvc-data/motor/identify_dynamics.py`
- Create (output): `C:/Users/tae06/CODE/tvc-data/motor/out/dynamics_identifiability.md`
- Test: `C:/Users/tae06/CODE/tvc-data/motor/test_identify_dynamics.py`

**Interfaces:**
- Produces:
  - `load_edges(run_dir) -> list[StepEdge]` (each: pre/post command, aligned Fz window, t relative to command change).
  - `varpro_fit(edges, tau_grid, free_onset=True) -> FitResult` (profiles baseline+amplitude linearly, scans shared τ, per-edge free onset).
  - `power_check(edges, true_taus, rng, n_trials) -> dict[tau -> recovered_tau_stats]` (inject known τ into synthetic steps built from real windows/noise/onset scatter; refit).
  - `main()` writes the report reproducing: near-hover τ non-identifiable (recovered≈0 for injected 44 ms), large-from-rest resolvable (~268 ms onset + τ≈96 ms, `T∝ω²` regime), and the 500 Hz / bigger-step recommendation.

- [ ] Step 1: Write failing test — `power_check` with an injected 44 ms τ under near-hover SNR returns median recovered τ ≈ 0 (asserts non-identifiability; guards against a future false "resolved").
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement extraction (thrust column = the tared Fz→thrust mapping used by `pwm_thrust_map.py`; confirm sign), varpro fit, power check, and report writer.
- [ ] Step 4: Run test + `python identify_dynamics.py`; verify test passes and report generates.
- [ ] Step 5: Commit (in tvc-data).

### Task 5: Update credibility doc row (tvc-testbed)

**Files:**
- Modify: `docs/7-CREDIBILITY.md` (Results Robustness motor row + the "three cheapest things" note)

- [ ] Step 1: Edit the motor delay/lag row to record: non-identifiable from existing 50 Hz data (host-clock command vs STM32 force ⇒ onset≡deadtime; SNR≈2.2 power-loss), answerable by a 500 Hz logging change (rig already averages ~20 raw samples/row), not a new rig. Keep status OPEN.
- [ ] Step 2: Commit.

---

## Module 3 — Battery sag

### Task 6: Battery identification from sustained runs (tvc-data)

**Files:**
- Create: `C:/Users/tae06/CODE/tvc-data/motor/identify_battery.py`
- Create (output): `C:/Users/tae06/CODE/tvc-data/motor/out/battery_sag.json`
- Test: `C:/Users/tae06/CODE/tvc-data/motor/test_identify_battery.py`

**Interfaces:**
- Produces:
  - `load_sustained(tests_dir) -> DataFrame` (concatenate the 7 A1850_B1850 runs, powered phases only, with cumulative mAh via coulomb counting of `current_a` over `t_epoch`).
  - `fit_sag(df) -> BatterySagFit`: `thrust_sensitivity_n_per_v`, `r`, `v_full`, `v_per_mah` (voltage vs drawn charge slope), `mah_capacity_est`, plus start/end thrust & voltage.
  - `main()` writes `battery_sag.json` and prints comparison to the YAML's current 1.46 N/V.

- [ ] Step 1: Write failing test — `fit_sag` on the sustained data returns `thrust_sensitivity_n_per_v` within a documented tolerance of the family (positive, order ~1–2 N/V) and `v_full`>`v_end`; coulomb integral monotonic increasing.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement loader, coulomb counting, least-squares fits.
- [ ] Step 4: Run test + script; verify.
- [ ] Step 5: Commit (tvc-data).

### Task 7: `BatteryState` model (tvc-testbed) + YAML update

**Files:**
- Create: `src/tvc_control/tvc_control/plant/battery.py`
- Modify: `vehicle_params.yaml` `motor_dynamics.sustained_load` (+ `v_per_mah`, `mah_capacity`, measured `thrust_sensitivity_n_per_v` if re-derived differs) with provenance note referencing the identification.
- Test: `tests/test_battery.py`

**Interfaces:**
- Produces:
  - `BatteryState(v_full, v_per_mah, capacity_mah, thrust_sensitivity_n_per_v)`.
  - `.update(current_a, dt) -> voltage_v` (coulomb counting).
  - `.derate(T_nominal) -> T_actual` = `T_nominal - k*(v_full - v_now)` clamped ≥ 0.
  - `.reset()`.

- [ ] Step 1: Write failing tests — (a) fresh pack (0 mAh drawn) ⇒ voltage=v_full, derate = identity; (b) after draining charge, voltage drops and derate reduces thrust; (c) monotonic; (d) reset restores.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement `BatteryState`; update YAML from `battery_sag.json` numbers with a `measured` provenance comment.
- [ ] Step 4: Run, verify pass.
- [ ] Step 5: Commit.

### Task 8: Wire battery into `ActuatorChain`/`SimConfig` (default off)

**Files:**
- Modify: `src/tvc_control/tvc_control/plant/actuators.py` (`ActuatorChain` optional `battery` + current estimate)
- Modify: `src/tvc_control/tvc_control/harness/mil.py` (`SimConfig.battery_sag: bool=False`, construct `BatteryState`, estimate current from thrust via `efficiency_g_per_w`/`peak_current_a`)
- Test: `tests/test_battery.py`, `tests/test_consistency.py`

**Interfaces:**
- Consumes: `BatteryState`. Current estimate: simple thrust→current map from YAML (`peak_current_a` at `T_max`), documented as approximate.

- [ ] Step 1: Write failing test — with `battery_sag=False`, a hover run's thrust trace is bit-identical to current `main`; with `battery_sag=True`, final thrust over a long hover is lower than initial.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement the optional path; when battery is None, `update` is unchanged.
- [ ] Step 4: Run full suite incl. goldens (`tvc.py hover --check-golden` unaffected: analytic MIL only), verify pass.
- [ ] Step 5: Commit.

---

## Module 4 — Aerodynamics

### Task 9: `aero.py` drag + ground effect

**Files:**
- Create: `src/tvc_control/tvc_control/plant/aero.py`
- Modify: `vehicle_params.yaml` add `aero:` block (`enabled: false`, `cd`, `ref_area_m2`, `air_density`, `rotor_radius_m`), all tagged estimated.
- Modify: `src/tvc_control/tvc_control/config.py` (expose aero block via `Vehicle.raw`/accessor; add fields to `VehicleParams`: `aero_enabled=False`, `cd=0.0`, `ref_area=0.0`, `rho=1.225`, `rotor_radius=0.0`).
- Test: `tests/test_aero.py`

**Interfaces:**
- Produces:
  - `drag_force_inertial(v_inertial, cd, ref_area, rho) -> np.ndarray` = `-0.5*rho*cd*ref_area*|v|*v`.
  - `ground_effect_factor(z, rotor_radius) -> float` = `1/(1-(R/4z)^2)` clamped (≥1, finite near ground), →1 as z→∞.

- [ ] Step 1: Write failing tests — drag opposes velocity, zero at rest, scales with v²; ground effect →1 at high z, >1 near ground, clamped finite at z→0⁺.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement both functions.
- [ ] Step 4: Run, verify pass.
- [ ] Step 5: Commit.

### Task 10: Integrate aero into `dynamics` (default off)

**Files:**
- Modify: `src/tvc_control/tvc_control/plant/rigidbody.py` (`dynamics` adds drag force + ground-effect thrust multiplier when `params.aero_enabled`)
- Test: `tests/test_aero.py`, `tests/test_consistency.py`

**Interfaces:**
- Consumes: `drag_force_inertial`, `ground_effect_factor`, `VehicleParams.aero_*`.

- [ ] Step 1: Write failing tests — (a) `aero_enabled=False` ⇒ state derivative bit-identical to current `dynamics`; (b) with drag on and no control/gravity, translational kinetic energy is non-increasing over an integration.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Add the conditional aero terms (drag on `v`; ground-effect factor multiplies `T` in the translational term only, documented).
- [ ] Step 4: Run full suite incl. goldens, verify pass.
- [ ] Step 5: Commit.

---

## Module 5 — Cross-plant validation suite (sequel)

### Task 11: Analytic-side replay + tolerance framework

**Files:**
- Create: `src/tvc_control/tvc_control/verify/cross_plant.py`
- Test: `tests/test_cross_plant.py`

**Interfaces:**
- Produces:
  - `ScenarioSpec` (init state + setpoints, one per current scenario) shared by both plants.
  - `run_analytic(spec, vp, gains) -> dict` metrics.
  - `compare(metrics_a, metrics_b, tolerances) -> list[Discrepancy]`.

- [ ] Step 1: Write failing tests — `compare` flags a metric outside tolerance and passes one inside; `run_analytic` reproduces the existing scenario metric for the lateral case.
- [ ] Step 2: Run, verify fail.
- [ ] Step 3: Implement specs, analytic replay, comparator. Gazebo runner stubbed behind a `run_gazebo(spec)` interface raising `NotImplementedError` until wired in the user's env.
- [ ] Step 4: Run, verify pass.
- [ ] Step 5: Commit.

### Task 12: Gazebo wiring + tolerance ratification (in user's Docker env)

**Files:**
- Modify: `src/tvc_control/tvc_control/verify/cross_plant.py`
- Modify: `docs/7-CREDIBILITY.md` (Verification/Validation rows if cross-plant agreement is achieved)

- [ ] Step 1: Verify Gazebo runs (`docker`/devcontainer). Implement `run_gazebo(spec)` via the existing gz harness.
- [ ] Step 2: Run all 5 scenarios both plants; ratify tolerances from repeated runs.
- [ ] Step 3: Record agreement (or the limit-cycle explanation) in the credibility doc.
- [ ] Step 4: Commit.

---

## Self-review notes

- Spec coverage: modules 1–5 all mapped to tasks; module-2 outcome is a null result (no YAML motor change), matching spec.
- Toggle-off bit-identity asserted in Tasks 1, 8, 10.
- Type consistency: `run_uncertainty`, `perturb_params`, `BatteryState`, `drag_force_inertial`, `ground_effect_factor`, `run_analytic`/`compare` names used consistently across tasks.
- Module 5 depends on Gazebo (user env) — Tasks 11 (native) and 12 (env) split accordingly.
