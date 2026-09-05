# gazebo/ — the physics-engine plant

The SDF model, the two worlds, and the script that runs a hover. Gazebo Harmonic
(gz-sim 8), inside the devcontainer.

```
gazebo/
  models/tvc_vehicle/
    model.sdf        GENERATED from vehicle_params.yaml -- do not hand-edit
    model.config     gz model metadata
    meshes/          tvc_vehicle.stl, the same mesh the 3D viewer renders
  worlds/
    tvc.sdf          ground plane, vehicle standing on its legs
    tvc_flight.sdf   airborne at 2 m and deliberately tilted; + a chase camera
  run_hover.sh       start paused, attach the controller, unpause
```

---

## Run it

```bash
bash gazebo/run_hover.sh                    # headless, 30 simulated seconds
bash gazebo/run_hover.sh --gui              # with the Gazebo window
bash gazebo/run_hover.sh --duration 60 --altitude 3.0 --log flight.csv
python tvc.py plot flight.csv
```

Bare Gazebo, no controller — the quickest check that the model loads and stands
on its legs:

```bash
GZ_SIM_RESOURCE_PATH="$PWD/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" gz sim -r gazebo/worlds/tvc.sdf
```

The full ROS 2 stack: `ros2 launch tvc_control gazebo.launch.py` — see
[../docs/6-RUNNING.md](../docs/6-RUNNING.md).

**Start paused, attach, then unpause.** Not cosmetic: the vehicle free-falls from
its 2 m spawn in about 0.6 s, so a controller connecting after an unpaused start
finds it already on the ground, and the plot looks exactly like a control
failure. Both `run_hover.sh` and `gazebo.launch.py` do this.

---

## model.sdf is generated

```bash
python tools/gen_model_sdf.py            # rewrite it
python tools/gen_model_sdf.py --check    # CI gate: fails if it has drifted
```

The hand-written SDF used to split the mass across six links whose placement made
the Gazebo **composite** CG land at 161 mm, while the mass tool and the controller
both assumed 211 mm. Gazebo was silently simulating a different vehicle than the
one the controller was tuned for, and nothing errored.

The generator removes that failure mode by construction: it **solves** for
`base_link`'s mass, pose and inertia so the composite of all links reproduces the
target `(mass, CG, full inertia tensor)` exactly. `--check` verifies it — current
CG error 2 × 10⁻¹⁶ mm.

Model choice: **single rigid `base_link`**. The gimbal rings and rotors carry only
small token masses needed for articulation and for the motor model, and those
token masses are *subtracted* from `base_link` rather than added on top, so the
total stays exact. Tilting-mass and rotor-gyroscopic effects are deliberately
minimised at this stage — what matters here is the thrust-authority-to-inertia
ratio and the gimbal geometry, and both are now exact.

---

## What is real and what is not

**Calibrated from bench data** (the `tvc-data` repository): the gimbal joint
limits, the mass properties, and — indirectly — the thrust and torque the vehicle
produces. Full provenance: [../docs/5-PARAMETERS.md](../docs/5-PARAMETERS.md).

**Geometry** is from `TVC Ver3.step`: gimbal pivot at the origin, rotors just
above, body stack to +652 mm, three legs at −78 mm. Collision shapes are
simplified cylinders rather than the CAD mesh — deliberate, since exact collision
meshes cost solver time and only matter on touchdown.

> ### ⚠ Simulated rotor speed is not an RPM
>
> `maxRotVelocity` and `momentConstant` in the SDF are **solver constants, not
> physics.** gz-sim's `MulticopterMotorModel` is separable, symmetric and linear
> in the thrust split; the measured coax surface is none of the three, and no
> single `momentConstant` can represent it — the required value spans 3.3× across
> the envelope, and being odd in the split it cannot express the surface's sign
> asymmetry at all.
>
> So the HAL inverts the plugin's own algebra on the command side and sends the
> ω pair that makes it produce exactly the measured `(T, τ_P)`. Exact to
> 7 × 10⁻¹⁵ N over the whole envelope. The price is that **ω is a control
> allocation variable**: nothing may read it as a physical speed, and the raised
> ceiling means the plugin no longer enforces the real 17.79 N thrust limit — the
> allocator does, and a test asserts it.
>
> [../docs/3-THEORY.md §8](../docs/3-THEORY.md) has the derivation;
> `tvc_control/hal/gazebo.py` has the code.

**The SDF servos are deliberately weak.** Gazebo's own `JointPositionController`
and joint velocity limits would add a *second* actuator lag on top of the one
`plant/actuators.py` models, and the two would not agree. Actuator dynamics are
owned in one place so that both plants model the same thing — which is what makes
an analytic-versus-Gazebo comparison mean anything.

---

## The worlds

| world | vehicle starts | for |
|---|---|---|
| `tvc.sdf` | on its legs, upright, on a ground plane | checking the model loads and stands |
| `tvc_flight.sdf` | airborne at 2 m, **deliberately tilted**, with a chase camera | every flight demo |

A perfectly upright spawn tests nothing: the vehicle sits in equilibrium, the
gimbal stays at exactly 0, and the plot looks flawless while proving nothing. And
the spawn height must clear the legs — the lowest leg point is z = −0.171 m in the
model frame, and spawning below that starts them inside the ground plane, where
the contact impulse throws the vehicle over.

**Takeoff from the ground is not solved.** Ground contact during spin-up is its
own problem: the legs are the only contact and the vehicle is tall and narrow.
Every scenario is deliberately airborne until then.

---

## Things that cost real debugging time

Each looked like a control problem and was not. The full list is in
[../docs/6-RUNNING.md](../docs/6-RUNNING.md); the two worst:

- **`OdometryPublisher` defaults to 2D.** Without `<dimensions>3</dimensions>`
  the message silently carries no `z` and no `vz`, so the altitude controller
  reads 0 forever and commands full thrust into the ground.
- **`gz` is a Ruby wrapper.** The real process has `comm == ruby`, so
  `pkill -x gz` matches nothing and dead simulators accumulate — and several
  publishing on the same topics feed the controller interleaved state from
  different worlds, which looks exactly like a physics instability. `pkill -f
  "gz sim"` is not the fix either: it matches the shell running the script, which
  then kills itself. `run_hover.sh` matches on `comm == ruby` **and** the
  arguments.
