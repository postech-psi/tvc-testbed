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
python tvc.py params          # every vehicle number, with its provenance
```

`tvc.py` is the only executable file at the top level. Everything else is a
library, an asset, or a document.

Then read, in order:

| | |
|---|---|
| [docs/1-ARCHITECTURE.md](docs/1-ARCHITECTURE.md) | the six layers, the two seams, and a file-by-file map |
| [docs/2-THEORY.md](docs/2-THEORY.md) | every equation the simulator implements, derived, with references |
| [docs/3-CONVENTIONS.md](docs/3-CONVENTIONS.md) | frames, axis names, signs, units — the single source |
| [docs/4-PARAMETERS.md](docs/4-PARAMETERS.md) | every number, where it came from, how much to trust it |
| [docs/5-RUNNING.md](docs/5-RUNNING.md) | the four pipelines, what each one is for, what its output means |
| [docs/6-CREDIBILITY.md](docs/6-CREDIBILITY.md) | **read before believing any result.** Validation is level 0 |
| [docs/7-ROADMAP.md](docs/7-ROADMAP.md) | what is next, what is deferred, what is genuinely unknown |

Supporting: [docs/MASS-BUDGET.md](docs/MASS-BUDGET.md) (where the mass and
inertia come from) and [docs/DEVCONTAINER.md](docs/DEVCONTAINER.md) (the Docker
environment, needed only for ROS 2 and Gazebo).

---

## The four pipelines

One controller, four ways to run it. The flight code in `gnc/` is **identical**
in all four — only the plant and the transport change, which is what makes a
result obtained in one of them evidence about the others.

| pipeline | command | plant | needs |
|---|---|---|---|
| **MIL** — analytic, fixed-step, deterministic | `python tvc.py validate` | `plant/` | Python |
| **Interactive** — same plant, with a form and a 3D view | `python tvc.py gui` | `plant/` | + a display |
| **SIL, no ROS** — gz-sim over gz-transport | `bash gazebo/run_hover.sh` | Gazebo | devcontainer |
| **SIL, full stack** — gz-sim + ROS 2 nodes | `ros2 launch tvc_control gazebo.launch.py` | Gazebo | devcontainer + `colcon build` |

A fifth, `ros2 launch tvc_control analytic.launch.py`, runs the ROS 2 nodes
against the analytic plant. It differs from the line above it only in which
plant process starts, which is how a failure gets attributed to the control code
or to the physics engine.

Full detail, including what each output means: [docs/5-RUNNING.md](docs/5-RUNNING.md).

---

## What is in the repository

```
tvc.py                 the entry point. --help lists everything.
docs/                  seven numbered documents; read them in order.
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

### Three rules the tests enforce

1. **`gnc/` imports only `math`, `dataclasses` and `typing`.** No numpy, no
   scipy, no file I/O, no `while` loops, no reach into `plant/`. That is what
   makes the eventual PX4 C++ module a port rather than a rewrite, and
   `tests/test_gnc_purity.py` parses the source to enforce it.
2. **Numbers live in one place.** `vehicle_params.yaml` is the source of truth;
   `model.sdf` and `docs/4-PARAMETERS.md` are generated from it and CI fails if
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

Read [docs/6-CREDIBILITY.md](docs/6-CREDIBILITY.md) before trusting output. The
headline, repeated wherever results are shown:

> **This simulator has never been compared against flight data.** It reproduces
> bench-measured *actuator* behaviour; it has not been shown to reproduce
> *vehicle* behaviour. Validation is level 0 of 4.

Two specific things to know:

- **The ROS 2 packages have never been built.** `colcon build` has not run
  against them. See [docs/5-RUNNING.md](docs/5-RUNNING.md) for the first-build
  procedure and what to expect.
- **The highest-value measurement outstanding** is whether the bench's 100 ms
  motor response is a transport delay or a first-order lag. Modelled both ways:
  as a lag the roll channel recovers cleanly; as a delay it winds up to ~72° and
  saturates 97% of the run. One bench run decides whether that channel is
  controllable at the current gains. `python tvc.py validate` prints both
  numbers on every run so the question cannot quietly stop being asked.
