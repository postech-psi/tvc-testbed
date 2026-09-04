# Conventions

**This file is the single source of truth for frames, axis names, signs, units
and identifiers.** Every other document links here and states no axis fact of
its own. If something contradicts this file, this file is right and the other
thing is a bug.

> ## Status: APPLIED
>
> The rename landed as one atomic commit. Every axis-bearing identifier in the
> code, the SDF, the message, the CSV and the ROS parameters now follows the
> convention below.
>
> **It moved no numbers.** The acceptance criterion was that `sim/golden/`
> survive the rename unchanged, and it did: 137 numeric values compared across
> the pre- and post-rename baselines under the key relabelling, zero
> differences. A pure rename that changes a number is not a rename, and without
> a frozen baseline that claim is untestable -- which is why the baseline was
> captured in Phase 0, before any of this work started.
>
> One real bug was caught in the attempt, by the test suite that was
> deliberately written first: the initial map omitted `yaw -> roll`, leaving a
> three-cycle two-thirds applied and producing
> `euler_to_quat(pitch, yaw, yaw)`. That is a SyntaxError, so it failed loudly
> -- but the same omission in a dict key or a topic name would not have.

---

## 1. Body frame

Origin at the centre of mass. **Body +z is the thrust axis and points up
through the airframe** (out the nose, along the props' axis when the gimbal is
centred). Body x and y are the lateral axes.

This frame does not move. The rename changes names only.

The vehicle has no aerodynamic reference direction and its three legs sit at
0°/120°/240°, so nothing physical distinguishes body x from body y. Body x is
defined as the leg-0 azimuth so that the choice is written down somewhere
rather than being implicit in a mesh.

---

## 2. Axis names — rocket convention

| name | body axis | rotation drives | gimbal ring | δ index |
|---|---|---|---|---|
| **roll** | **+z** (thrust axis) | τ_P — differential prop reaction torque | — | — |
| **pitch** | **x** | gimbal | **outer** | δ₂ |
| **yaw** | **y** | gimbal | **inner** | δ₁ |

### Why this assignment and not the other one

Pitch and yaw are physically interchangeable on this airframe — a 90° roll maps
one onto the other and nothing observable changes. So the assignment cannot be
settled on physical grounds, and exactly one non-arbitrary criterion remains:
the named triad must be **right-handed**, `roll × pitch = yaw`, which is what
REP-103 and every mixer and rotation-matrix in the stack assumes.

- Adopted: `ẑ × x̂ = ŷ` ✅
- Rejected: pitch = y, yaw = x → `ẑ × ŷ = −x̂` ❌ left-handed

The rejected option looks cheaper (it renames one SDF joint instead of two) but
it names a triad in which "positive roll then positive pitch gives *negative*
yaw" — a trap for every future reader and for any PX4 interop.

The adopted assignment agrees independently with the controller-design note's
own table (`x_B, y_B → pitch, yaw`).

### Why roll is the thrust axis

Standard for launch vehicles: roll is rotation about the long/thrust axis. It
is *not* standard for multicopters, and PX4 is a multicopter stack — see §4.

---

## 3. Gimbal

Chain, outermost first:

```
base_link → [outer ring joint, axis 1 0 0, body x] → outer_gimbal
          → [inner ring joint, axis 0 1 0, body y] → inner_gimbal → rotors
```

The **ring** names (`inner` / `outer`) are mechanical facts and are invariant
under any axis convention. That is why the SDF joints and the gz topics are
named after rings, not axes (§6): if the convention is ever revised again, the
mechanical layer does not move.

> `inner` / `outer` mean **the gimbal ring, and only that**. The cascaded
> control loops are `attitude loop` (outer) and `rate loop` (inner) — never
> "inner/outer loop".

Thrust direction in the body frame, exact (not small-angle):

```
n̂(δ₁, δ₂) = [ sin δ₁ , −sin δ₂ cos δ₁ , cos δ₁ cos δ₂ ]
```

with δ₁ the inner (yaw-plane) deflection and δ₂ the outer (pitch-plane) one.

Moments, with the thrust applied at `r_G = (0, 0, −L)` from the CM and the props
riding on the gimbal so their reaction torque tilts with the thrust:

```
M_pitch (x) = −T·L·sin δ₂ cos δ₁  +  τ_P·sin δ₁
M_yaw   (y) = −T·L·sin δ₁         −  τ_P·sin δ₂ cos δ₁
M_roll  (z) =                        τ_P·cos δ₁ cos δ₂
```

Read off this: the gimbal has **no** authority about the thrust axis, τ_P is the
only source of roll, and lateral authority is proportional to `T·L` — so it
falls with thrust during descent.

Measured travel is **asymmetric and differs per ring**; a symmetric ±7° model
overstates negative travel on both:

| ring | plane | travel | slew | −3 dB BW |
|---|---|---|---|---|
| inner | yaw | −6.46° … +6.98° | 403 °/s | 13 Hz |
| outer | pitch | −6.77° … +6.86° | 235 °/s | 9 Hz |

Both rings: ~30 ms transport deadtime (onset 10 ms, delay 30.5 ms, rise₁₀₋₉₀
42 ms, t_s±2% 280 ms).

---

## 4. PX4 boundary — the translation table

**PX4 uses the FRD body frame (Forward-Right-Down) and calls rotation about the
thrust axis `yaw`. We call it `roll`.** Both are standard in their own domain
(launch vehicle vs multicopter) and PX4's names are not ours to change.

Our frame is PX4's rotated 180° about x:

| ours (rocket, +z = thrust axis, up) | PX4 (FRD, +z down) |
|---|---|
| body x | x_FRD (same) |
| body y | **−**y_FRD |
| body z (thrust axis) | **−**z_FRD |
| **roll** (about thrust axis) | **yaw** |
| **pitch** (about body x) | **roll** |
| **yaw** (about body y) | **−pitch** |

**Without this table, integration silently flips the sign of two axes out of
three.** It is owned by `hal_px4` alone and asserted by `tests/test_px4_frame.py`
(round-trip identity plus the sign of each axis). FRD never appears anywhere
inside `tvc_gnc/`.

Relevant PX4 interface: with `OffboardControlMode.direct_actuator = true`, PX4
disables its own control allocator and the external controller supplies
`ActuatorMotors` / `ActuatorServos` directly (normalized, FRD). This is the same
uORB interface a future in-tree PX4 module would publish, which is why the
flight code's actuator boundary is shaped like it (§5).

---

## 5. Layer boundaries

```
tvc_gnc/    flight code — runs on the vehicle. No plant, no I/O, no clock.
tvc_plant/  simulation only — rigid body, actuator dynamics, sensors.
harness/    owns the clock and the transport (MIL / SIL / later PIL, HIL).
```

Two seams, and only two:

**Seam A — `EstimatedState`.** The controller consumes an *estimate*, never
ground truth, even today when the estimator is the identity. Carries
`pos_i, vel_i, quat (body←inertial), omega_b, stamp_s`. `stamp_s` exists now
because estimator delay is certain to appear later and adding the field then
would touch every call site.

**Seam B — `ActuatorSetpoint`.** `motor_a, motor_b` normalized [0,1] against the
surface's `pwm_min`/`pwm_max`; `gimbal_pitch_rad`, `gimbal_yaw_rad`; plus the
predicted `thrust_n`, `tau_p_nm` and the saturation flags for logging and
anti-windup. Motors normalized and gimbal in radians is deliberate: each is the
natural coordinate of its own calibration.

The measured-surface inverse lives in `tvc_gnc/allocation.py`, **not** in a HAL
— the feasible set comes from the surface, so an allocator that cannot see it
will command what the vehicle cannot produce.

---

## 6. Identifier map

Applied in one atomic commit. The left column is what the repo used to say;
it is kept because a reader hitting an old branch, an old log or an old plot
needs to be able to translate it, and because §7's guards are written against
these exact strings.

### Externally visible — rename together or not at all

| kind | old | new |
|---|---|---|
| SDF joint | `gimbal_roll_joint` | `gimbal_outer_joint` |
| SDF joint | `gimbal_pitch_joint` | `gimbal_inner_joint` |
| gz/ROS topic | `/tvc_vehicle/gimbal_roll` | `/tvc_vehicle/gimbal_outer_cmd` |
| gz/ROS topic | `/tvc_vehicle/gimbal_pitch` | `/tvc_vehicle/gimbal_inner_cmd` |
| msg field | `delta1` | `gimbal_inner_rad` (body y, yaw plane) |
| msg field | `delta2` | `gimbal_outer_rad` (body x, pitch plane) |
| ROS param | `roll_des_deg` | `att_pitch_des_deg` ⚠ **meaning changes** |
| ROS param | `pitch_des_deg` | `att_yaw_des_deg` ⚠ **meaning changes** |
| ROS param | `axial_des_deg` | `att_roll_des_deg` |
| ROS param | `init_roll_deg` | `init_att_pitch_deg` ⚠ |
| ROS param | `init_pitch_deg` | `init_att_yaw_deg` ⚠ |
| ROS param | `gimbal_rate_max_deg` | **retired** — a scalar summary of a per-axis fact, and already silently ignored |
| CSV header | `roll_deg,pitch_deg,yaw_deg` | `roll_deg,pitch_deg,yaw_deg` ⚠ **same names, reordered and remapped** |
| CSV header | `gimbal_roll_deg,gimbal_pitch_deg` | `gimbal_outer_deg,gimbal_inner_deg` |
| metrics key | `final_roll_deg` / `final_pitch_deg` / `final_axial_deg` | `final_pitch_deg` / `final_yaw_deg` / `final_roll_deg` ⚠ |
| metrics key | `max_delta1_deg` / `max_delta2_deg` | `max_gimbal_inner_deg` / `max_gimbal_outer_deg` |

⚠ marks a **value swap, not a rename**: the identifier survives with a
different meaning. Those are the dangerous ones, and they are why §7 exists.

### Internal

| old | new |
|---|---|
| `pid_roll_angle` / `pid_pitch_angle` / `pid_axial_angle` | `pid_pitch_angle` / `pid_yaw_angle` / `pid_roll_angle` ⚠ |
| `dp` / `dr` (hover.py) | `d_inner` / `d_outer` |
| "axial channel" | "roll channel" |
| "lateral pair" | "pitch/yaw pair" |
| "outer loop" / "inner loop" | "attitude loop" / "rate loop" |

---

## 7. Three guards against the silent trap

The failure mode this whole section prevents: an identifier that survives the
rename while its meaning changes. Nothing errors; the numbers are just wrong.

**(a) No live external identifier is reused.** The SDF joints and gz topics move
to *ring* names, which collide with nothing old. If a name must survive (CSV
attitude columns), the file carries a convention token and old files are
rejected, not reinterpreted.

**(b) Retired ROS parameters hard-error.** Each node checks the retired list at
startup and exits with a message pointing here. `SimConfig` drops the fields and
replaces them with same-named properties whose getter *and* setter raise.

**(c) A convention token travels with the data.** `axis_convention: rocket_v2`
in `sim/vehicle_params.yaml` (loader raises if absent or unknown), a
`uint8 axis_convention` field in the actuator message, and a header comment in
the CSV that `plot_flight.py` checks before plotting.

### Renaming mechanics

**Never run a substring `s/roll/…/`.** The string `controller` contains `roll`.
About half of the ~387 raw hits in this repo are false positives:
`controller_node`, `AttitudeController`, `AltitudeController`, `ControllerNode`,
`JointPositionController`, `gz-sim-joint-position-controller-system`,
`ros-jazzy-ros2-controllers`, `controllable`, `uncontrollable`, `scrollbar`,
`yscrollcommand`, `scrollregion`, `yview_scroll`, `rollingMomentCoefficient`,
`rolling_moment_coefficient`, `microcontroller`.

Use the explicit token map above with `\b` anchors plus
`tools/rename_denylist.txt`, then verify that

```
git grep -nE '\broll\b|\bpitch\b|\byaw\b|\bdelta[12]\b|\baxial\b'
```

returns only denylisted hits.

---

## 8. Units and sign conventions

| quantity | unit | note |
|---|---|---|
| length | m | except YAML mass-properties, which are mm and marked as such |
| mass / inertia | kg / kg·m² | inertia about the CM, **full tensor** — products are not negligible: `Iyz/Izz = 27.3%` |
| angle | rad in code, deg only at UI and YAML boundaries | field names carry `_deg` when they are degrees |
| force / moment | N / N·m | |
| PWM | µs | the coordinate the bench characterization is defined in |
| time | s | |

- Attitude is a unit quaternion `(qw, qx, qy, qz)`, body←inertial. ROS
  `geometry_msgs/Quaternion` is `(x, y, z, w)` — reorder at the boundary.
- Euler angles are **readout only**. Never propagate kinematics through them.
- τ_P > 0 when prop B's drag torque exceeds prop A's. Verified consistent with
  the measured surface (`∂T_z/∂b = +0.087` vs `∂T_z/∂a = −0.080`) and with
  Gazebo's `τ = momentConstant·(T_b − T_a)`. This chain currently agrees *by
  luck*; `tests/test_axis_convention.py` asserts it.
- Positive δ₂ (outer/pitch plane) produces positive `M_pitch`. Asserted, not
  assumed — both position-loop signs in `hover.py` were wrong once.

---

## 9. Vehicle numbers

Do not copy these into other documents; link here. The table is regenerated
from `sim/vehicle_params.yaml` by `tools/gen_docs.py` (Phase 7), and until then
is hand-maintained.

| quantity | value |
|---|---|
| mass | 1.3280 kg |
| inertia (CM) | `Ixx` 0.022616, `Iyy` 0.022581, `Izz` 0.001957 kg·m² |
| products | `Ixy` −2.2e−5, `Ixz` −3.01e−4, `Iyz` −5.35e−4 kg·m² |
| lever arm L (pivot → CM) | 0.2111 m |
| max thrust | 17.79 N → T/W 1.37, hover at 73 % throttle |
| τ_P at hover | −0.089 … +0.147 N·m (asymmetric; 0.089 guaranteed both ways) |
| lateral authority at hover | 0.309 N·m → 13.7 rad/s² |
| roll authority at hover | 0.089 N·m → 45.4 rad/s² (`Izz` is 11.6× smaller) |

Axial authority is **not** the quoted bench peak of 0.18 N·m — that occurs at
A=2000/B=1000 where total thrust is 10.4 N, *below* hover. It collapses toward
zero as thrust approaches the ceiling.

---

## References

- PX4 [Simulation](https://docs.px4.io/main/en/simulation/) and
  [Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard) —
  flight-code/simulator boundary, lockstep, `direct_actuator`
- ROS [REP-103](https://www.ros.org/reps/rep-0103.html) — frame and unit conventions
- T. A. Johansen, T. I. Fossen, *Control allocation — A survey*, Automatica 49(5), 2013
