# tvc-testbed

Simulator and flight code for a **coaxial thrust-vector-controlled VTVL
demonstrator** — POSTECH UGRP 2026. Two counter-rotating propellers on a 2-axis
gimbal, 1.33 kg, T/W 1.37. The goal is a vehicle that takes off vertically,
hovers under thrust vector control, and lands: a testbed for reusable-launch-
vehicle GNC that the next team can extend rather than rebuild.

This repository answers *what will the vehicle do, and what code flies it*. The
companion repository `tvc-data` answers *what does the hardware actually do*, and
the dependency runs one way: bench measurements parameterise this model, never
the reverse.

---

## Start here

```bash
python tvc.py --help          # every runnable thing in this repository
python tvc.py validate        # 5 closed-loop scenarios, ~20 s, no dependencies
python tvc.py trace           # every computation in one control step, with numbers
python tvc.py params          # every vehicle number, with its provenance
```

`tvc.py` is the only executable file at the top level. Everything else is a
library, an asset, or a document.

Then read, in order:

| | |
|---|---|
| [docs/1-CODE-MAP.md](docs/1-CODE-MAP.md) | **what every directory, file and function is** — the index is generated from the source |
| [docs/2-WALKTHROUGH.md](docs/2-WALKTHROUGH.md) | **what actually happens when it runs**, stage by stage, with the real numbers |
| [docs/3-THEORY.md](docs/3-THEORY.md) | every equation the simulator implements, derived, with references |
| [docs/4-CONVENTIONS.md](docs/4-CONVENTIONS.md) | frames, axis names, signs, units — the single source |
| [docs/5-PARAMETERS.md](docs/5-PARAMETERS.md) | every number, where it came from, how much to trust it |
| [docs/6-RUNNING.md](docs/6-RUNNING.md) | the four execution paths, what each is for, what its output means |
| [docs/7-CREDIBILITY.md](docs/7-CREDIBILITY.md) | **read before believing any result.** Validation is level 0 |
| [docs/8-ROADMAP.md](docs/8-ROADMAP.md) | what is next, what is deferred, what is genuinely unknown |

Supporting: [docs/MASS-BUDGET.md](docs/MASS-BUDGET.md) (where the mass and
inertia come from) and [docs/DEVCONTAINER.md](docs/DEVCONTAINER.md) (the Docker
environment, needed only for ROS 2 and Gazebo). Before editing, read
[CONTRIBUTING.md](CONTRIBUTING.md) for the source-to-generated-file map and the
complete check list.

---

## The four execution paths

One controller, two plants, with and without ROS 2. The flight code in `gnc/`
is **identical** in all four paths — only the plant and transport change. That
controlled variation is what lets you attribute a disagreement.

| pipeline | command | plant | needs |
|---|---|---|---|
| **MIL** — analytic, fixed-step, deterministic | `python tvc.py validate` | `plant/` | Python |
| **ROS analytic** — ROS 2 nodes against the analytic plant | `ros2 launch tvc_control analytic.launch.py` | `plant/` | devcontainer + `colcon build` |
| **SIL, no ROS** — gz-sim over gz-transport | `bash gazebo/run_hover.sh` | Gazebo | devcontainer |
| **ROS SIL** — gz-sim + ROS 2 nodes | `ros2 launch tvc_control gazebo.launch.py` | Gazebo | devcontainer + `colcon build` |

`python tvc.py gui` and `python tvc.py view3d` are views over the MIL path, not
additional controller or plant implementations.

Full detail, including what each output means: [docs/6-RUNNING.md](docs/6-RUNNING.md).

---

## What is in the repository

```
tvc.py                 the entry point. --help lists everything.
docs/                  eight numbered documents; read them in order.
src/
  tvc_control/         the ROS 2 python package -- ALL the code lives here
    tvc_control/
      gnc/             FLIGHT CODE. Runs on the vehicle. Pure Python by rule.
      plant/           SIMULATION ONLY. Rigid body, actuator lag, sensors.
      hal/             transport adapters. No control.
      harness/         owns the clock and the I/O. mil.py and gz.py.
      nodes/           ROS 2 wrappers. Thin: no control math.
      apps/            GUI, 3D viewer, plotter, GIF recorder.
      verify/          the scenario suite and the frozen baseline.
      config.py        the ONLY place vehicle numbers enter the program.
      vehicle_params.yaml   what the hardware IS (measured)
      control_gains.yaml    what we CHOSE (tuned)
    launch/            two launch files, differing only in the plant.
  tvc_msgs/            one message: ActuatorCommand.
gazebo/                the SDF model, the worlds, and run_hover.sh.
tools/                 generators: CAD -> mass properties -> parameters -> SDF.
tests/                 the merge gate. Pure Python, no ROS, no Gazebo.
reference/             frozen baselines, the PX4 airframe, a servo sketch.
```

The double `src/tvc_control/tvc_control/` is ROS 2's convention, not a mistake:
a colcon package directory contains a Python package of the same name.

### Three design rules

1. **`gnc/` imports only `math`, `dataclasses` and `typing`.** No numpy, no
   scipy, no file I/O, no unbounded work, no reach into `plant/`. That keeps an
   eventual PX4 C++ module a port rather than a rewrite. Review this boundary
   directly when changing `gnc/`; it is intentionally not hidden in meta-tests.
2. **Numbers live in one place.** `vehicle_params.yaml` is the source of truth;
   `model.sdf` and `docs/5-PARAMETERS.md` are generated from it and CI fails if
   either has drifted. There is no fallback constant anywhere — a missing YAML
   is an error, because defaults that can disagree with a measurement eventually
   do. (A superseded 20 N max thrust once survived in three files after the
   bench measured 17.79 N.)
3. **One controller.** Every pipeline reaches the control law through
   `TvcController`. `tests/test_consistency.py` fails if a gain constant appears
   anywhere outside `control_gains.yaml`.

---

## Checks

All three run in CI, and the first is the merge gate.

```bash
python -m pytest tests/ -q         # purity, conventions, allocation, scenarios
python tvc.py golden --check       # the frozen numerical baseline
python tools/gen_model_sdf.py --check   # the Gazebo model matches the parameters
python tools/gen_docs.py --check        # the documented numbers match the YAML
```

---

## The honest status

Read [docs/7-CREDIBILITY.md](docs/7-CREDIBILITY.md) before trusting output. The
headline, repeated wherever results are shown:

> **This simulator has never been compared against flight data.** It reproduces
> bench-measured *actuator* behaviour; it has not been shown to reproduce
> *vehicle* behaviour. Validation is level 0 of 4.

Two specific things to know:

- **The two software-in-the-loop paths disagree in one channel.** Both fly, both
  hold altitude and lateral attitude, and both run byte-identical control code —
  but under ROS 2 the thrust axis holds a 1.5–2.5° limit cycle that the direct
  gz-transport path settles out of entirely. That is the channel with 11.5× less
  inertia and the slowest actuator, so the bridge's transport delay lands
  exactly where there is least margin. Bounded, attributable, unexplained. See
  [docs/7-CREDIBILITY.md](docs/7-CREDIBILITY.md).
- **The vehicle cannot hold roll attitude at full throttle.** Above ~17.0 N of
  the 17.79 N ceiling the reachable propeller-torque interval no longer contains
  zero, so a climb at maximum thrust is *forced* to apply roll torque and the
  thrust axis takes 53° before the throttle comes off the stop. Real behaviour,
  correctly modelled, and nothing yet stops a climb profile from doing it.
- **The highest-value measurement outstanding** is whether the bench's 100 ms
  motor response is a transport delay or a first-order lag. Modelled both ways:
  as a lag the roll channel recovers cleanly; as a delay it winds up to ~72° and
  saturates 97% of the run. One bench run decides whether that channel is
  controllable at the current gains. `python tvc.py validate` prints both
  numbers on every run so the question cannot quietly stop being asked.
