# 3 — Conventions

**This file is the single source of truth for frames, axis names, signs and
units.** Every other document links here and states no axis fact of its own. If
something contradicts this file, this file is right and the other thing is a bug.

Vehicle *numbers* live in [4-PARAMETERS.md](4-PARAMETERS.md), which is generated
from the YAML. Do not copy either into a third place.

---

## 1. Body frame

Origin at the centre of mass. **Body +z is the thrust axis and points up through
the airframe** — out the nose, along the propellers' axis when the gimbal is
centred. Body x and y are the lateral axes.

The vehicle has no aerodynamic reference direction and its three legs sit at
0°/120°/240°, so nothing physical distinguishes body x from body y. Body x is
*defined* as the leg-0 azimuth, so that the choice is written down somewhere
rather than being implicit in a mesh.

The inertial frame is ENU: +z up, gravity `−g ẑ`.

---

## 2. Axis names — the rocket convention

| name | body axis | what drives it | gimbal ring | δ index |
|---|---|---|---|---|
| **roll** | **+z** (thrust axis) | τ_P — differential prop reaction torque | — | — |
| **pitch** | **x** | gimbal | **outer** | δ₂ |
| **yaw** | **y** | gimbal | **inner** | δ₁ |

### Why roll is the thrust axis

Standard for launch vehicles: roll is rotation about the long/thrust axis. It is
*not* standard for multicopters, and PX4 is a multicopter stack — see §4.

### Why pitch = x and not y

Pitch and yaw are physically interchangeable on this airframe: a 90° roll maps
one onto the other and nothing observable changes. So the assignment cannot be
settled on physical grounds, and exactly one non-arbitrary criterion remains —
**the named triad must be right-handed**, `roll × pitch = yaw`, which REP-103 and
every mixer and rotation matrix in the stack assumes.

- Adopted: `ẑ × x̂ = ŷ` ✅
- Rejected: pitch = y, yaw = x → `ẑ × ŷ = −x̂` ❌ left-handed

The rejected option looks cheaper (it renames one SDF joint instead of two) but
it names a triad in which *"positive roll then positive pitch gives negative
yaw"* — a trap for every future reader and for any PX4 interop.

Asserted in `tests/test_axis_convention.py::test_named_triad_is_right_handed`.

---

## 3. Gimbal

Joint chain, outermost first:

```
base_link → [gimbal_outer_joint, axis 1 0 0, body x] → outer_gimbal
          → [gimbal_inner_joint, axis 0 1 0, body y] → inner_gimbal → rotors
```

The **ring** names (`inner` / `outer`) are mechanical facts and are invariant
under any axis convention. That is why the SDF joints and the gz topics are named
after rings, not axes: if the convention is ever revised again, the mechanical
layer does not move, and no live external identifier is reused with a new
meaning.

> `inner` / `outer` mean **the gimbal ring, and only that**. The cascaded control
> loops are the **attitude loop** and the **rate loop** — never "inner/outer
> loop".

Thrust direction and the moments it produces are in
[2-THEORY.md §3](2-THEORY.md). Measured travel is asymmetric and differs per
ring; the numbers are in [4-PARAMETERS.md](4-PARAMETERS.md).

---

## 4. The PX4 boundary — the translation table

**PX4 uses the FRD body frame (Forward-Right-Down) and calls rotation about the
thrust axis `yaw`. We call it `roll`.** Both are standard in their own domain —
launch vehicle versus multicopter — and PX4's names are not ours to change.

Our frame is PX4's rotated 180° about x:

| ours (rocket, +z = thrust axis, up) | PX4 (FRD, +z down) |
|---|---|
| body x | x_FRD (same) |
| body y | **−**y_FRD |
| body z (thrust axis) | **−**z_FRD |
| **roll** (about the thrust axis) | **yaw** |
| **pitch** (about body x) | **roll** |
| **yaw** (about body y) | **−pitch** |

**Without this table, integration silently flips the sign of two axes out of
three.** It will be owned by a PX4 HAL alone and asserted by a round-trip test.
FRD appears nowhere inside `gnc/`.

Relevant interface: with `OffboardControlMode.direct_actuator = true`, PX4
disables its own control allocator and the external controller supplies
`ActuatorMotors` / `ActuatorServos` directly — normalized, FRD. This is the same
uORB interface a future in-tree PX4 module would publish, which is why the flight
code's actuator boundary is shaped like it
([1-ARCHITECTURE.md §2](1-ARCHITECTURE.md)).

---

## 5. Units and signs

| quantity | unit | note |
|---|---|---|
| length | m | except the YAML mass-properties block, which is mm and marked as such |
| mass / inertia | kg / kg·m² | inertia about the CM, **full tensor** — the products are not negligible: `Iyz/Izz = 27.3%` |
| angle | rad in code; deg only at UI and YAML boundaries | a field name carries `_deg` when it is degrees |
| force / moment | N / N·m | |
| PWM | µs | the coordinate the bench characterization is defined in |
| normalized command | [0, 1] motors, [−1, 1] servos | against each actuator's own calibrated range |
| time | s | |

- Attitude is a unit quaternion **`(qw, qx, qy, qz)`, body ← inertial**. ROS
  `geometry_msgs/Quaternion` is `(x, y, z, w)` — reorder at the boundary. Both
  ROS nodes do; `gz.msgs.Odometry` uses named fields so no reorder is needed.
- **Euler angles are readout only.** Never propagate kinematics through them.
  `quat_to_euler` returns `(pitch, yaw, roll)` — ZYX, in that order.
- **τ_P > 0** when prop B's drag torque exceeds prop A's. Consistent with the
  measured surface (`∂T_z/∂b = +0.087` against `∂T_z/∂a = −0.080`) and with
  Gazebo's `τ = momentConstant·(T_b − T_a)`. Three independent choices agree here
  **by coincidence**, so `tests/test_consistency.py` asserts the chain end to end
  rather than trusting it.
- **Positive δ₂ (outer ring, pitch plane) produces positive `M_pitch`.** Asserted,
  not assumed — both position-loop signs were wrong once.
- Rotor A is the CCW rotor, `motorNumber 0`; rotor B is CW.

---

## 6. Guards against the silent trap

The failure mode this whole file exists to prevent: **an identifier that survives
a rename while its meaning changes.** Nothing errors; the numbers are just wrong.

**(a) No live external identifier is reused.** SDF joints and gz topics are named
after *rings*, which collide with nothing. Where a name must survive — the CSV
attitude columns are still `roll_deg`, `pitch_deg`, `yaw_deg` — the file carries a
convention token and old files are **rejected, not reinterpreted**
(`apps/plot.py`).

**(b) Retired parameters hard-error.** Each ROS node checks a `RETIRED_PARAMS`
list at startup and exits with a message pointing here. A parameter that looks
live while being ignored is worse than one that is gone: it makes a launch file
document a control decision that is not happening. Currently retired:

| retired | why | replacement |
|---|---|---|
| `gimbal_rate_max_deg` | a scalar summary of a per-ring fact, and it was already being silently ignored | `gimbal.axes.*.rate_max_deg` in the YAML |
| `roll_des_deg`, `pitch_des_deg`, `axial_des_deg` | pre-rename names whose **meaning** changed | `att_pitch_des_deg`, `att_yaw_des_deg`, `att_roll_des_deg` |
| `init_roll_deg`, `init_pitch_deg`, `init_axial_deg` | same | `init_att_pitch_deg`, `init_att_yaw_deg`, `init_att_roll_deg` |

**(c) A convention token travels with the data.** `axis_convention: rocket_v2` in
`vehicle_params.yaml` and `control_gains.yaml` (the loader raises if absent or
unknown), a `uint8 axis_convention` field in `ActuatorCommand` where **0 means
unset** so a default-constructed message fails the receiver's check, and a header
line in every flight-log CSV that the plotter verifies before drawing.

### If the convention is ever revised again

1. Never run a substring `s/roll/…/`. The string `controller` contains `roll`, and
   about half of the raw hits in this repository are false positives:
   `controller_node`, `AttitudeController`, `AltitudeController`,
   `JointPositionController`, `gz-sim-joint-position-controller-system`,
   `ros-jazzy-ros2-controllers`, `controllable`, `scrollbar`, `yscrollcommand`,
   `rollingMomentCoefficient`, `microcontroller`.
2. Freeze a numerical baseline first (`python tvc.py golden`). A pure rename moves
   no numbers, and without a record taken beforehand that claim is untestable —
   which is exactly how a rename quietly becomes a physics change.
3. Write the assertions **before** the rename. When this convention was adopted,
   the map omitted `yaw → roll`, leaving a three-cycle two-thirds applied and
   producing `euler_to_quat(pitch, yaw, yaw)`. That is a `SyntaxError`, so it
   failed loudly — but the same omission in a dict key or a topic name would not
   have.
4. Afterwards, check the baseline is unchanged. When this convention was adopted,
   137 numeric values were compared under the key relabelling: zero differences.

### Reading an old branch, log or plot

| old | new | note |
|---|---|---|
| `gimbal_roll_joint` / `gimbal_pitch_joint` | `gimbal_outer_joint` / `gimbal_inner_joint` | |
| `/tvc_vehicle/gimbal_roll` / `…_pitch` | `/tvc_vehicle/gimbal_outer_cmd` / `…_inner_cmd` | |
| `delta1` / `delta2` fields | `gimbal_inner_rad` / `gimbal_outer_rad` | |
| `tvc_msgs/GimbalCommand` | `tvc_msgs/ActuatorCommand` | carried only two angles; thrust and τ_P were computed and discarded |
| `/ctrl/gimbal_cmd` | `/ctrl/actuator_cmd` | |
| `/sim/vehicle_attitude` (`sensor_msgs/Imu`) | `/model/tvc_vehicle/odometry` (`nav_msgs/Odometry`) | an IMU carries no position or velocity, so no outer loop could close |
| CSV `roll_deg, pitch_deg, yaw_deg` | same names, **different axes** | ⚠ this is why the token exists |
| metrics `final_roll_deg` / `final_pitch_deg` / `final_axial_deg` | `final_pitch_deg` / `final_yaw_deg` / `final_roll_deg` | ⚠ value swap, not a rename |
| "axial channel" | "roll channel" | |
| "outer loop" / "inner loop" | "attitude loop" / "rate loop" | |

⚠ marks the dangerous cases: the identifier survives with a different meaning.

---

## References

- ROS [REP-103](https://www.ros.org/reps/rep-0103.html) — frame and unit conventions
- PX4 [Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard) —
  `direct_actuator`, `ActuatorMotors` / `ActuatorServos`, the FRD frame
