# 2 — Theory

Every equation the simulator implements, where it comes from, and which file
contains it. Axis names and signs follow [4-CONVENTIONS.md](4-CONVENTIONS.md)
throughout; numbers are from [5-PARAMETERS.md](5-PARAMETERS.md).

**The one-paragraph version.** The vehicle is a rigid body with a single
gimballed thrust vector applied below its centre of mass. Tilting that vector
gives torque about the two lateral axes; it gives *none* about the thrust axis
itself, so the only roll authority is the differential drag reaction of two
counter-rotating propellers. Attitude and altitude are closed with cascaded PID
loops; the map from the loops' output (a body moment and a total thrust) to
actuator commands is a separate allocation layer, and that layer is where the
vehicle's real awkwardness lives — the reachable set of (thrust, roll torque) is
a curved, asymmetric lens rather than a box.

---

## 1. The vehicle, and what is modelled

| | |
|---|---|
| mass | 1.328 kg |
| max thrust | 17.79 N → T/W = 1.37, hover at **73% throttle** |
| inertia | `Ixx` 0.02262, `Iyy` 0.02258, **`Izz` 0.001957** kg·m² |
| lever arm `L` | 0.2111 m, gimbal pivot → CM |
| gimbal travel | inner −6.46…+6.98°, outer −6.77…+6.86° |

`Izz` is **11.6× smaller** than `Ixx`. Almost everything surprising about this
vehicle follows from that one ratio and from the fact that the gimbal cannot
touch that axis.

**Modelled:** 6-DOF rigid-body dynamics with the full inertia tensor; the exact
(not small-angle) gimballed thrust direction; the measured two-input
thrust/torque surface; gimbal transport delay and per-ring slew limits; motor
response lag.

**Not modelled, and it matters:** aerodynamics of any kind (no drag, no ground
effect, no wind); battery voltage sag and the −13% thrust derate it causes over
a flight; servo resonance (+11 dB inner, +5 dB outer); gimbal hysteresis (up to
0.65°); sensor noise, bias and latency; ground contact. Each is listed with its
consequence in [7-CREDIBILITY.md](7-CREDIBILITY.md).

---

## 2. Rigid-body dynamics

`plant/rigidbody.py`. Standard Newton–Euler for a rigid body, 13 states:
position `r`, velocity `v` (both inertial), attitude quaternion `q` (body ←
inertial), body rates `ω`.

### Translation

$$\dot{\mathbf v} = \frac{1}{m}\,\mathbf R(q)\,\mathbf F_b + \begin{bmatrix}0\\0\\-g\end{bmatrix},
\qquad \mathbf F_b = T\,\hat{\mathbf n}(\delta_1,\delta_2)$$

### Attitude kinematics

$$\dot q = \tfrac12\,\Omega(\boldsymbol\omega)\,q$$

Linear in `q` and singularity-free — which is the reason attitude is propagated
as a quaternion and Euler angles are **readout only**. `q` is renormalized after
every integration step; truncation error accumulates otherwise.

### Rotation

$$\mathbf I\dot{\boldsymbol\omega} = \boldsymbol\tau - \boldsymbol\omega\times(\mathbf I\boldsymbol\omega)$$

solved with the **full** tensor, not the diagonal:

$$\mathbf I = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz}\end{bmatrix}$$

This is not fastidiousness. `Iyz/Izz = 27.3%` and `Ixz/Izz = 15.4%` on this
airframe, so a diagonal approximation *dominates* the thrust-axis response — and
Gazebo carries the full tensor in the SDF, so keeping the diagonal would make
the two plants disagree for a reason having nothing to do with the control model.

---

## 3. The thrust vector and the moments it produces

This section is the core of the vehicle model. `gnc/mathx.py::thrust_axis` and
`plant/rigidbody.py::dynamics`.

### Thrust direction

The gimbal is a two-ring chain: `base_link → outer ring (about body x) → inner
ring (about body y) → rotors`. Composing the two rotations exactly:

$$\hat{\mathbf n}(\delta_1,\delta_2) = \begin{bmatrix} \sin\delta_1 \\ -\sin\delta_2\cos\delta_1 \\ \cos\delta_1\cos\delta_2 \end{bmatrix}$$

with δ₁ the **inner** ring (yaw-plane) deflection and δ₂ the **outer** ring
(pitch-plane) one. Exact rather than small-angle: at 7° the small-angle error is
only ~0.3%, but writing it exactly costs nothing and removes an approximation
from the argument entirely.

### Moments

Thrust acts at `r_G = (0, 0, −L)` from the CM. The propellers ride **on the
gimbal**, so their net reaction torque `τ_P` tilts with the thrust rather than
staying aligned with body z:

$$\boldsymbol\tau = \mathbf r_G \times (T\,\hat{\mathbf n}) + \tau_P\,\hat{\mathbf n}$$

Expanding:

$$\begin{aligned}
M_{\text{pitch}}\ (x) &= -T L \sin\delta_2\cos\delta_1 \;+\; \tau_P\sin\delta_1 \\
M_{\text{yaw}}\ \ (y) &= -T L \sin\delta_1 \;-\; \tau_P\sin\delta_2\cos\delta_1 \\
M_{\text{roll}}\ (z) &= \phantom{-T L \sin\delta_1\;} \tau_P\cos\delta_1\cos\delta_2
\end{aligned}$$

**Three facts read directly off these equations, and they shape everything else:**

1. **The gimbal has no authority about the thrust axis.** Every term in `M_roll`
   carries `τ_P`. This is a structural property of `r × F` with a purely axial
   `r` — not an approximation — and `tests/test_axis_convention.py` asserts it
   for a grid of gimbal angles.
2. **`τ_P` is not absent from the lateral axes.** It contributes
   `τ_P sin δ₁` and `−τ_P sin δ₂ cos δ₁`. Dropping those terms (which earlier
   code did) under-predicts lateral torque by ~6.5% of full authority once roll
   is commanded, and aims the gimbal `atan(τ_P / TL)` off the intended torque
   axis — 16° of rotation, measured against only 6.5° of available travel.
3. **Lateral authority is proportional to `T·L`.** It therefore *falls with
   thrust during descent*, exactly when a landing vehicle needs it.

### The pendulum rocket fallacy

Thrust applied below the CM does **not** make the vehicle self-stabilising, and
it does not make it diverge like an inverted pendulum either. With the gimbal
centred, `τ = r_G × F = 0` regardless of orientation, and gravity acts at the CM
and makes no torque at all. **Attitude is a double integrator — neutrally
stable.** A pendulum has a ground pivot for gravity to act about; a flying
vehicle does not.

What actually diverges is *position*: any tilt puts a horizontal component on
the thrust vector, which accelerates the vehicle sideways, which needs an
opposite tilt to arrest. That is the loop that runs away, and it is why the
position loop is deliberately slow (§6).

---

## 4. Control allocation

`gnc/allocation.py`. The layer that turns a desired body moment and total thrust
into actuator commands. Johansen & Fossen's hierarchy: *motion control → virtual
control effort → allocation → effector commands*.

### The lateral 2×2

Under small angles the lateral moment equations become linear in (δ₁, δ₂):

$$\begin{bmatrix} M_x \\ M_y \end{bmatrix} = \begin{bmatrix} \tau_P & -TL \\ -TL & -\tau_P \end{bmatrix}\begin{bmatrix} \delta_1 \\ \delta_2 \end{bmatrix}$$

The determinant is `−(τ_P² + (TL)²) < 0` for **any** thrust and any roll command,
so the inverse always exists — **there is no allocation singularity to guard
against, only actuator limits.** The matrix squares to `(τ_P² + (TL)²)·I`, so it
is its own inverse up to that scalar; the code uses Cramer's rule on the 2×2
rather than a library solve, which at this size is the same arithmetic minus the
dependency and minus the unbounded-work worry.

That seed is then refined by **three Newton steps on the exact trigonometric
map** — cheap insurance that removes the small-angle approximation entirely, and
that would matter if the travel were ever revised upward.

### The roll/gimbal coupling, closed by iteration

`M_z` can only come from `τ_P`, but what reaches body z is `τ_P cos δ₁ cos δ₂`,
and δ₁, δ₂ are not known until the lateral stage has run — which in turn needs
`τ_P`. One extra pass closes the loop: solve the gimbal against the first `τ_P`
guess, divide the guess by the cosine loss those angles imply, re-solve.

Skipping it leaves `M_z` short by `τ_P(1 − cos δ₁ cos δ₂)`, which at the 7° stops
is 1.5% of the roll command. Small — but it is a *systematic bias*, not noise,
so the roll integrator would spend the whole flight paying it off.

### Saturation scales the pair, it does not clip the axes

When the gimbal hits its stops, the solution is scaled by the largest
`s ∈ (0, 1]` that keeps **both** axes inside their own (asymmetric, per-ring)
travel. Clipping each axis independently would rotate the commanded torque
vector toward the corner of the box — which is exactly the wrong thing to do
while recovering from an off-axis upset. Measured: on a lopsided request at
double the available authority, scaling rotates the realized torque by <0.5°
while per-axis clipping would rotate it by ~19°.

### The feasible set

`τ_P` is bought with a thrust split, so the reachable `(T, τ_P)` region is the
image of the command square under the measured surface — a curved lens.
Consequences:

- roll authority **peaks near half throttle** and vanishes at both ends;
- it is **asymmetric**: at hover, `τ_P ∈ [−0.089, +0.147] N·m`.

The allocator clamps to the *signed* interval and reports `roll_saturated`. The
conservative symmetric figure — `min(|lo|, |hi|)` = 0.0888 N·m, guaranteed in
both directions — is what authority claims and gain sizing use.

> The headline bench figure of **0.18 N·m** is not available at hover. It occurs
> at A=2000/B=1000, where total thrust is 10.4 N — *below* hover — and it
> collapses toward zero as thrust approaches the ceiling.

---

## 5. Actuator effectiveness — the measured surface

`gnc/effectiveness.py`. The bench result the whole model rests on: a 121-point
sweep of both motor commands (1000–2000 µs in 100 µs steps, 4 s dwell, 50 Hz;
first 1 s of each dwell discarded for overshoot, a dwell rejected when the
least-squares slope over its last 3 s exceeded 0.3). 119 points survived.

Both outputs are fitted as bivariate cubics in the normalized commands
`a = (A − 1500)/500`, `b = (B − 1500)/500`:

$$f(a,b) = \sum_{i+j\le 3} c_{ij}\,a^i b^j$$

| | R² | RMSE |
|---|---|---|
| thrust `f_T` | 0.9984 | 0.243 N |
| roll torque `f_Q` | 0.9965 | 0.0032 N·m |

Confirmed against two independently quoted anchors: `f_T(1,1) = 17.79 N` (the
peak thrust) and `f_Q(1,−1) = −0.173 N·m` (the peak reaction torque). Both are
asserted in `tests/test_effectiveness.py`.

### Why a surface and not two per-motor curves

**In a coaxial pair the lower rotor runs inside the upper rotor's wake**, so its
thrust *and* its drag torque depend on both commands. `T = T₁(a) + T₂(b)` does
not hold, and the fit says so plainly:

- the mixed terms are large: `−0.978·ab` in thrust, `−0.479·a²b`;
- mirrored commands are not mirrored results: `+0.180 N·m` at `(−1,+1)` against
  `−0.173 N·m` at `(+1,−1)`, with 11.18 N of thrust against 10.37 N.

**That asymmetry *is* the wake**, and it is the reason a single "moment constant"
cannot represent this vehicle (§8).

### The inverse

Given a desired `(T, τ_P)`, find the command pair. Newton on the 2×2 Jacobian,
warm-started from the previous solution — consecutive control commands differ by
a fraction of a newton, so the previous answer is already inside Newton's basin
and converges in two or three steps. The cold path uses a coarse 33×33 seed
table, because this surface is not monotone in `b` and a fixed seed can converge
onto the wrong branch near the edges.

**Thrust has priority, and enforcing that takes more than clamping the torque.**
The boundary of the reachable set is exactly where the Jacobian degenerates, so a
target sitting *on* it is the one case Newton cannot solve: it stalls with a large
thrust residual and would silently return a command producing over a newton less
lift than asked. So the solve is a ladder — try the requested torque, then 97%,
90%, 70%, 40%, 0% of it — ending in a **bisection along the balanced diagonal
`a = b`**, where

$$f_T(t,t) = 6.876 + 8.844\,t + 1.945\,t^2 + 0.129\,t^3$$

is strictly increasing on `[−1, 1]` (its derivative `8.844 + 3.890t + 0.387t²` is
positive throughout) and spans the entire thrust range. That last rung cannot
fail, so **thrust is always achieved exactly**; what is given up is roll
authority.

Every stage carries a fixed iteration bound, so the worst-case work is a number
someone can write down.

---

## 6. The control loops

`gnc/controller.py` runs them in this order, and the order is forced:

1. **position** → attitude setpoint — produces a *setpoint*, not an effort, so
   it belongs outside the attitude loop entirely
2. **altitude** → thrust command — must precede allocation, because the feasible
   set is a function of `T`
3. **attitude + rate** → body moment
4. **allocation** → actuator commands

All gains below are the `flight_validated` profile
(`control_gains.yaml`), the only set that has been near a plant.

### Attitude and rate — the cascade

$$\boldsymbol\omega_{des} = K_{p,\text{angle}}\,\mathbf e_q, \qquad
\dot{\boldsymbol\omega}_{des} = \text{PID}(\boldsymbol\omega_{des} - \boldsymbol\omega), \qquad
\mathbf M = \mathbf I_{\text{diag}}\,\dot{\boldsymbol\omega}_{des}$$

**The attitude error is the quaternion form**, not an Euler difference:

$$\mathbf e_q = 2\,\text{sgn}(q_{e,w})\,\mathbf q_{e,v}, \qquad q_e = q^{-1}\otimes q_{des}$$

The `sgn` picks the short way round — without it the controller happily takes the
350° path to a 10° target, which on a vehicle with 7° of gimbal travel is not a
slow recovery but a tumble. The two forms agree to second order: measured,
`|e_q − Δeuler| ≤ 0.21·|θ|²`, so 0.1% of the angle at 1° and 4.7% at the 8° the
position loop may command (`tests/test_attitude_error.py`). The Euler form was
replaced because its arcsin singularity sat on one specific axis, which meant the
axis *names* carried a stability caveat; the quaternion form has no preferred
axis, so the caveat disappears rather than moving.

**The rate loop outputs angular acceleration, not torque.** Inertia is applied in
exactly one place. This looks cosmetic and is not: two historical gain sets wrote
`τ = I·K·e` and `τ = k·e` respectively, so the same name meant two quantities
differing by a factor of `I` and they appeared 24.9× apart while describing
nearly the same loop. In normalized units they can be put side by side, which is
what made adopting the flown set possible.

The **diagonal** inertia is used for this mapping even though the plant
integrates the full tensor. That is deliberate: this is gain scheduling, not
dynamics, and a controller that inverts its own model's cross terms is claiming a
model accuracy the mass budget does not support.

Since the plant is a double integrator, the closed loop is
`s² + K_rate·s + K_rate·K_angle`:

| axis | `K_angle` | `K_rate` [1/s] | ω_n | ζ |
|---|---|---|---|---|
| pitch / yaw | 5.0 | 22.0 | **10.5 rad/s** (1.67 Hz) | 1.05 |
| roll | 2.0 | 229.9 | **21.4 rad/s** (3.41 Hz) | 5.36 |

The roll rate gain looks alarming and is not: `Izz · 229.9 = 0.450 N·m/(rad/s)`
against the lateral `Ixx · 22.0 = 0.498`. **The large number is a small inertia,
not aggression.** An earlier revision hand-scaled the roll gain by `Izz/Ixx`,
which meant re-measuring `Izz` would silently have changed the loop bandwidth;
now the scaling is explicit and follows the measurement.

### Altitude, and the 1/cos θ feedforward

$$v_{z,des} = \text{clamp}(K_{p,alt}(z_{des}-z),\ \pm v_{z,max}), \qquad
a_{z,des} = \text{PID}(v_{z,des}-v_z)$$

$$T = \frac{m\,(g + a_{z,des})}{\max(\hat{\mathbf z}\cdot \mathbf R\hat{\mathbf n},\ \cos_{min})}$$

Vertical dynamics are `m·z̈ = T·(R n̂)·ẑ − mg`, so the useful fraction of thrust
is `cos θ`. Dividing the command by that same projection is **feedforward**: it
cancels the altitude sag from a tilt at the instant the tilt happens, instead of
waiting for the error to grow enough for feedback to notice. Measured effect:
**84% less peak sag** at a 6° commanded tilt (1.0 mm against 6.1 mm).

The divisor is floored at `cos_min = 0.7` so a large or briefly bad attitude
estimate cannot demand unbounded thrust; at the floor the vehicle simply accepts
the sag, which is the safe failure.

### Position hold, and why it must be slow

$$\theta_{yaw,des} = -\text{clamp}(K_p e_x + K_d v_x), \qquad
\theta_{pitch,des} = +\text{clamp}(K_p e_y + K_d v_y)$$

The opposite signs are not a typo. Rotating body +z into the world, **+yaw tips
thrust toward +x and +pitch tips it toward −y**, so returning to the origin needs
opposite-signed demands. Getting either backwards turns the loop into positive
feedback and the vehicle accelerates away; both were wrong once, and the
derivation is preserved verbatim in `gnc/position.py` because that comment is
what caught it.

Closing on `a = g·θ` gives `s² + g·K_d·s + g·K_p`, i.e. **ω_n = 0.99 rad/s,
ζ = 0.99** at the current 0.10/0.20 — about **10× slower** than the attitude
loop. That separation is the whole design. At an earlier 0.22/0.35 the two loops
were only 6.5× apart and the vehicle settled into a **±9° coning limit cycle** at
the attitude natural frequency, pitch and yaw 90° out of phase.

### Anti-windup

Every loop's integrator freezes while the actuator it drives is saturated
(`Allocation.gimbal_saturated`, `roll_saturated`, `thrust_saturated`). The output
cannot follow anyway, so continuing to integrate only builds a charge that must
be repaid as overshoot when the limit clears. Proportional and derivative terms
still respond.

---

## 7. Actuator dynamics — and the open question

`plant/actuators.py`. Simulation only.

### Gimbal: two different lags, often confused

| | value | effect |
|---|---|---|
| **rate limit** | 403 °/s inner, 235 °/s outer | how fast the servo slews once moving |
| **transport deadtime** | 30 ms, both rings | how long before it moves at all |

The deadtime is the one that costs phase margin. It is *pure delay*, so it eats
`ω·T_d` radians at every frequency and cannot be compensated by a lead term the
way a first-order lag can. At the lateral crossover of 10.5 rad/s that is **18°
of phase**. It is modelled as a FIFO of commands rather than a filter, because
that is what a serial servo bus actually does: the command sits in a queue, then
executes in full.

### Motors: the highest-value measurement outstanding

The bench recorded "command → thrust response delay ≈ 0.10 s, averaged over 105
step transitions" — and, unlike the gimbal block which separates onset / delay /
rise₁₀₋₉₀ / settling, **it does not say whether that is a transport delay or a
first-order time constant.**

That distinction decides whether the roll channel is controllable at the current
gains. At the roll loop's 21.4 rad/s:

| reading | phase cost | measured outcome, 20° roll upset |
|---|---|---|
| first-order lag, τ = 0.10 s | `atan(ωτ)` = **65°** | recovers cleanly, 0% saturated |
| pure transport delay, 0.10 s | `ωT_d` = **123°** | winds up to **72°**, 97% saturated |

The lateral axes are unaffected either way — this is entirely a roll-channel
result, and it follows from `Izz` being 11.6× smaller while carrying the stiffest
normalized gain.

The model is therefore **switchable** (`motor_dynamics.model:
first_order | delay | delay_plus_lag`) with the optimistic reading as default,
and `python tvc.py validate` prints the pessimistic number on every run so the
question cannot quietly stop being asked. **One bench run resolves it.**

The lag acts on the physical pair `(T, τ_P)`, not on rotor speed, because that is
what the bench measured — and since `T ∝ ω²`, a first-order lag in ω is not a
first-order lag in thrust anyway.

---

## 8. Why Gazebo needed a command-side inversion

`hal/gazebo.py`. gz-sim's `MulticopterMotorModel` computes, per rotor,

$$T_i = k\,\omega_i^2, \qquad \tau_i = \pm c\,T_i
\quad\Longrightarrow\quad T = k(\omega_a^2+\omega_b^2), \quad \tau_P = c\,(T_b - T_a)$$

**Separable, symmetric, linear in the split. The measured surface is none of the
three.** At hover, the plugin with the original `c = 0.016` produced only ~50% of
the measured roll torque.

**Fitting `c` cannot work**, for two independent reasons. The value required to
cover the measured envelope runs from 0.016 at low thrust to 0.054 near the
ceiling — a 3.3× spread. And more fundamentally, `c` appears as an **odd**
function of the split, so it *cannot represent sign asymmetry at all*: fitting
the middle over-promises the weak direction by ~65% at hover, the controller
commands a torque the plant cannot produce, the roll integrator winds up, and the
symptom looks like a tuning problem rather than a modelling one. Per-rotor
asymmetric constants are worse still — unequal `k` means equal ω no longer gives
equal thrust, so the vehicle produces a parasitic pitch torque at zero pitch
command.

**The fix: solve the plugin's own algebra for the ω pair that makes it produce
exactly the `(T, τ_P)` the allocator chose off the measured surface.** Two
equations, two unknowns, closed form:

$$s = \tau_P/c, \qquad \omega_a = \sqrt{\tfrac{(T-s)/2}{k}}, \qquad \omega_b = \sqrt{\tfrac{(T+s)/2}{k}}$$

Exact to 7×10⁻¹⁵ N over the whole measured envelope, and the asymmetry is fully
reproduced — because it lives in the *commanded* `τ_P`, which came from the
surface, and not in the plugin.

**What this costs, stated plainly.** Rotor angular velocity in Gazebo stops being
a physical RPM: it is a control-allocation variable, `momentConstant` becomes a
solver scaling constant and `maxRotVelocity` becomes solver headroom. Nothing may
read ω as physics. And since the raised ceiling means the plugin no longer
enforces the vehicle's real 17.79 N limit, **the allocator does** — asserted by
test rather than assumed.

Feasibility drove the SDF change: at the original `maxRotVelocity = 1100` the
required `c` diverges near full thrust so no single value works; at 1200+ it
converges to 0.0323, so the model uses **1300 and c = 0.04** for margin. Checked
across the envelope: hover with τ_P = 0 → (941, 941); the strong direction →
(798, 1066); the weak → (1019, 857); full thrust → (1100, 1100). All inside 1300.

---

## 9. Numerical methods

| where | method | bound |
|---|---|---|
| plant integration | `scipy.integrate.solve_ivp`, RK45, `max_step = dt/4`, zero-order-hold control | adaptive; simulation only |
| quaternion | renormalized every step | — |
| lateral gimbal inverse | Cramer's rule seed + 3 Newton steps on the exact map | fixed 3 |
| surface inverse | warm-started Newton, 33×33 seed table for the cold path | ≤ 40 steps × 6 ladder rungs |
| surface inverse, last resort | bisection on the balanced diagonal | fixed 48 |
| feasible-set table | 161×161 sweep at construction, binned into 200 thrust bins | startup only |

The 161×161 construction sweep is *startup* work, not control-path work — the
same thing a flash-resident table would be. The seed table is deliberately much
coarser (33×33) because it has to live in flash: the full sweep would be ~830 KB
against ~35 KB.

**Determinism.** Every scenario is a fixed-step ZOH loop with a fixed `max_step`
and no randomness, so repeated runs agree to the last bit. That is what makes
`reference/golden/` meaningful — and if it ever stops being true,
`tvc.py golden --check` says so rather than drifting quietly.

---

## References

The repository's control design follows these; where a choice differs from them,
the reason is stated at the point of difference.

- **T. Lajarte**, *Guidance, Navigation and Control of a Sounding Rocket* (2021)
  — the GNC architecture and the axial-lever-arm TVC moment model of §3
- **Spannagl et al.**, EmboRockETH (2021) — gimbal geometry and the cascaded PID
  structure of §6
- **Linsen et al.**, *Optimal thrust vector control of an electric small-scale
  rocket* (2022)
- **T. A. Johansen & T. I. Fossen**, *Control allocation — a survey*,
  Automatica 49(5):1087–1103, 2013 — the allocation hierarchy of §4
- **NASA-STD-7009**, *Standard for Models and Simulations* — the credibility
  framework of [7-CREDIBILITY.md](7-CREDIBILITY.md)
- **R. G. Sargent**, *Verification and validation of simulation models*, WSC 2010
- **ROS REP-103** — frame and unit conventions
- **PX4**, [Simulation](https://docs.px4.io/main/en/simulation/) and
  [Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard)
