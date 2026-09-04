# 4 — Parameters

Every number the simulator uses, what it is, and **how much of a claim it is**.

> **The tables below are generated** from
> `src/tvc_control/tvc_control/vehicle_params.yaml` by `tools/gen_docs.py`, and
> CI fails if they have drifted. Do not hand-edit inside the `EMIT` markers, and
> do not copy these numbers into a third document — link here instead.
>
> ```bash
> python tvc.py params            # the same table, in a terminal
> python tools/gen_docs.py        # regenerate this file after a YAML change
> ```

---

## The two parameter files, and why they are two

| file | answers | changes when |
|---|---|---|
| `vehicle_params.yaml` | **what is the hardware?** | the airframe is re-measured or rebuilt |
| `control_gains.yaml` | **what did we choose?** | someone tunes |

Mixing them would mean a tuning experiment dirties the file whose provenance
discipline is its entire value. The mass-properties block of the first is
machine-written by `tools/mass_properties.py --emit`; everything else in it is
hand-maintained bench data.

**There is no third place.** No fallback constants exist anywhere: a missing YAML
raises. That rule is not fastidiousness — a superseded 20.0 N max thrust once
survived in three separate files after the bench measured 17.79 N, and a
180 °/s gimbal slew outlived its measurement at 235/403 °/s. Nothing failed. The
simulation just quietly described a different vehicle.

---

## How to read the `source` column

| tag | means | how far to trust it |
|---|---|---|
| **measured** | from a bench run, with a stated fit error | the strongest claim here |
| **derived** | computed from measured or CAD inputs | as good as its inputs |
| **CAD** | from the STEP/STL model — *design intent*, not metrology | the built vehicle has never been weighed or balanced |
| **estimated** | someone's assumption; no measurement exists | ⚠ treat conclusions resting on these as provisional |
| **chosen** | a modelling or control decision, not a property | not a claim about the world at all |
| **solver** | a constant that exists to make a numeric method work | **must not be read as physics** |

<!-- <<<EMIT:provenance -->
| provenance | count | share |
|---|---|---|
| measured | 16 | 46% |
| derived | 10 | 29% |
| chosen | 4 | 11% |
| CAD | 2 | 6% |
| solver | 2 | 6% |
| estimated | 1 | 3% |
| **total** | **35** | |
<!-- >>>EMIT:provenance -->

---

## Control authority — what the vehicle can actually do

The numbers gain choices rest on. Everything here is a product of parameters that
move when the airframe is re-measured, which is why it is generated rather than
written.

<!-- <<<EMIT:authority -->
| quantity | value | how it is obtained |
|---|---|---|
| hover thrust | 13.03 N (73% of the 17.79 N ceiling) | m*g |
| lateral moment | 0.3094 N*m | T*L*sin(travel), travel = 6.46 deg, the tighter stop |
| lateral angular accel | 13.68 rad/s^2 | lateral moment / Ixx |
| roll moment, guaranteed both ways | 0.0888 N*m | the measured feasible set at hover thrust |
| roll moment, signed interval | -0.0888 .. +0.1473 N*m | asymmetric: the coax wake makes one direction stronger |
| roll angular accel | 45.38 rad/s^2 | roll moment / Izz, and Izz is 11.6x smaller than Ixx |
| roll : lateral accel ratio | 3.3x | authority-rich, but through a 3x slower actuator |
| tau_P cross-coupling | 3.2% of T*L | rotates the allocation by 1.85 deg, 29% of the tighter axis's travel |

Roll authority against total thrust -- it peaks near half throttle and
collapses at both ends, because tau_P is bought with a thrust split:

| thrust | % of ceiling | reachable tau_P |
|---|---|---|
| 5.34 N | 30% | -0.0873 .. +0.0842 N*m |
| 8.89 N | 50% | -0.1447 .. +0.1391 N*m |
| 11.56 N | 65% | -0.1244 .. +0.1714 N*m |
| 13.03 N (hover) | 73% | -0.0888 .. +0.1473 N*m |
| 15.12 N | 85% | -0.0426 .. +0.0976 N*m |
| 16.90 N | 95% | -0.0025 .. +0.0463 N*m |
<!-- >>>EMIT:authority -->

Three things to take from this table:

1. **Roll is authority-rich and bandwidth-poor.** It gets ~3× the angular
   acceleration of the lateral pair — because `Izz` is 11.6× smaller — but it
   acts through a ~100 ms motor response against the gimbal's ~30 ms. The
   inertia ratio alone gives exactly the wrong intuition.
2. **Lateral authority is proportional to `T·L`**, so it *falls during descent*,
   which is when a landing vehicle needs it.
3. **`L` is the least trustworthy number in the whole model.** It sets all
   lateral authority and it has never been measured — it comes from a CAD CG in
   which over half the mass sits at assumed positions.

---

## The parameters

<!-- <<<EMIT:parameters -->
### Mass properties

| quantity | value | unit | source | note |
|---|---|---|---|---|
| mass | `1.3280` | kg | **CAD** | components.yaml + a lumped remainder to the 1328 g design total |
| CG x | `-1.7` | mm | **derived** | computed from the itemized masses |
| CG y | `-1.0` | mm | **derived** | computed from the itemized masses |
| CG z | `+211.1` | mm | **derived** | computed from the itemized masses |
| Ixx | `0.022616` | kg*m^2 | **derived** | mesh inertia for CAD solids, m*d^2 for point masses |
| Iyy | `0.022581` | kg*m^2 | **derived** | mesh inertia for CAD solids, m*d^2 for point masses |
| Izz | `0.001957` | kg*m^2 | **derived** | mesh inertia for CAD solids, m*d^2 for point masses |
| Ixy | `-0.000022` | kg*m^2 | **derived** | product of inertia, carried in full |
| Ixz | `-0.000301` | kg*m^2 | **derived** | product of inertia, carried in full |
| Iyz | `-0.000535` | kg*m^2 | **derived** | product of inertia -- NOT negligible here: Iyz is 27% of Izz |

### Geometry

| quantity | value | unit | source | note |
|---|---|---|---|---|
| lever arm L | `0.2111` | m | **estimated** | mode=pivot; UNMEASURED and it sets all lateral authority |
| rotor plane z | `45.0` | mm | **CAD** | where the multicopter plugin applies thrust |

### Gimbal

| quantity | value | unit | source | note |
|---|---|---|---|---|
| travel (nominal) | `+/-7.0` | deg | **measured** | symmetric summary; the SDF joint stops use it |
| inner travel | `-6.46 .. +6.98` | deg | **measured** | yaw plane (body y); asymmetric about its own neutral |
| inner slew | `403` | deg/s | **measured** | peak measured rate |
| inner bandwidth | `13` | Hz | **measured** | -3 dB; resonance +11 dB is NOT modelled |
| inner angle fit | `RMSE 0.197` | deg | **measured** | R2 0.9977, hysteresis up to 0.43 deg (unmodelled) |
| outer travel | `-6.77 .. +6.86` | deg | **measured** | pitch plane (body x); asymmetric about its own neutral |
| outer slew | `235` | deg/s | **measured** | peak measured rate |
| outer bandwidth | `9` | Hz | **measured** | -3 dB; resonance +5 dB is NOT modelled |
| outer angle fit | `RMSE 0.312` | deg | **measured** | R2 0.9942, hysteresis up to 0.65 deg (unmodelled) |
| transport deadtime | `30` | ms | **measured** | pure delay; costs phase at every frequency |

### Rotors

| quantity | value | unit | source | note |
|---|---|---|---|---|
| max thrust | `17.79` | N | **measured** | f_T(1,1) of the surface; T/W = 1.37 |
| motor_constant | `7.35e-06` | N/(rad/s)^2 | **derived** | from max thrust; used by the gz plugin |
| moment_constant | `0.04` | m | **solver** | NOT a drag ratio -- scaling for the Gazebo command-side inversion |
| max_rot_velocity | `1300` | rad/s | **solver** | solver headroom, not a physical RPM limit |
| rotor drag / rolling | `0.0` | - | **chosen** | zeroed for plant parity: they scale with a fictitious omega |

### Thrust/torque surface

| quantity | value | unit | source | note |
|---|---|---|---|---|
| form | `cubic_ab` | - | **measured** | cubic in a=(A-1500)/500, b=(B-1500)/500 |
| thrust fit | `RMSE 0.243` | N | **measured** | R2 0.9984 over 119 of 121 swept points |
| torque fit | `RMSE 0.0032` | N*m | **measured** | R2 0.9965; fresh-pack, no derating modelled |

### Motor dynamics

| quantity | value | unit | source | note |
|---|---|---|---|---|
| response | `100` | ms | **measured** | OPEN: delay or lag is not recorded, and it decides roll controllability |
| model | `first_order` | - | **chosen** | the optimistic reading of the line above |
| tau_s | `0.100` | s | **chosen** | first-order time constant on (T, tau_P) |
| sustained derate | `13.8 -> 12.0` | N | **measured** | over 7 x 60 s; recorded, NOT modelled |

### Environment

| quantity | value | unit | source | note |
|---|---|---|---|---|
| gravity | `9.81` | m/s^2 | **chosen** |  |
<!-- >>>EMIT:parameters -->

---

## Control gains

Not generated — these are choices, and the file's comments record the tuning
history that justifies them. Two profiles exist; `flight_validated` is the
default.

| | `flight_validated` | `analytic_legacy` |
|---|---|---|
| provenance | flown in Gazebo, repeatedly | never flown, never tuned against a plant |
| attitude `kp_angle` / `kp_rate` | 5.0 / 22.0 | 4.0 / 0.884 |
| roll `kp_angle` / `kp_rate` | 2.0 / 229.9 | 4.0 / 0.920 |
| altitude `kp_alt` / `kp_vz` | 4.0 / 3.0 | 1.5 / 4.0 |
| integral terms | **all zero** | present |
| known weakness | 0.86° steady-state error | 66° roll excursion on a ground-start climb, ~25 s to recover |

**Units.** The rate loop outputs *angular acceleration* [rad/s² per rad/s], not
torque; inertia is applied once, inside `AttitudeController`. This is what makes
the two sets comparable at all — they were previously written in different units
and appeared 24.9× apart while describing nearly the same loop. See
[2-THEORY.md §6](2-THEORY.md).

**A prediction that did not come true, recorded because it was made.** The worry
when adopting `flight_validated` was that 30 ms of modelled gimbal deadtime —
which the Gazebo demo never modelled — would destabilise it. It does not: an A/B
over lateral upset, roll upset and a ground climb shows no divergence and
essentially no difference between 30 ms and 0 ms. It also halves the climb
transient the analytic set suffers (66.4° → 36.9° peak). The cost is steady-state
error, 0.86° against 0.25°, because this set has `ki = 0` on every axis. That is
a real regression on a real metric and it is the next tuning question — not
something to paper over by inventing an integral gain that has never flown.

---

## What is recorded but deliberately not modelled

These numbers live in `vehicle_params.yaml` so that enabling them later is a code
change and not a re-measurement. Each one is a known divergence between this
simulator and the vehicle; see
[6-CREDIBILITY.md](6-CREDIBILITY.md#results-robustness) for the consequences.

| measured | value | why it is not modelled |
|---|---|---|
| battery sag / thrust derate | 11.9 → 10.0 V, 13.8 → 12.0 N over 7 × 60 s | the surface is a fresh-pack fit; it describes the start of a flight, not the end |
| servo resonance | +11 dB inner, +5 dB outer | a rate limit cannot represent a lightly damped peak; closed-loop margin near it is unmodelled |
| gimbal hysteresis | 0.24°/0.43° inner, 0.35°/0.65° outer | up to 9% of the outer ring's travel |
| phase lag at 2 Hz | 40° inner, 53° outer | superseded by the deadtime + slew model, kept for cross-checking it |
| thrust sensitivity to pack voltage | 1.46 N/V, r = 0.981 over 11.2–12.5 V | would need a battery state model |
| peak current, efficiency | 34.7 A, 4.9 g/W | no electrical model exists |

---

## Two open parameter decisions

**`geometry.lever_arm_mode`** is `pivot` (`L = cg_z = 0.2111 m`), meaning thrust
is modelled as acting at the gimbal pivot. The alternative, `rotor_plane`
(`L = cg_z − rotor_z = 0.1661 m`), is where Gazebo actually applies it. **The
choice is worth 21% of all lateral authority** and it is still deferred. Nothing
in the bench data settles it; a measurement of where the thrust resultant acts
would.

**The 100 ms motor response** is recorded as `response_delay_s: 0.10` but the
bench did not say whether it is a transport delay or a first-order time constant.
`motor_dynamics.model` selects between them, defaulting to the optimistic
reading. This is the single highest-value measurement outstanding — see
[2-THEORY.md §7](2-THEORY.md).
