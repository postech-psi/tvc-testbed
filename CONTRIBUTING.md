# Contributing

Start with [README.md](README.md), then use [docs/1-CODE-MAP.md](docs/1-CODE-MAP.md)
to locate the layer you need. Before changing a sign, frame, parameter, or
claimed result, read the corresponding numbered document; those details are
part of the interface, not background commentary.

## Change the source, then regenerate its dependants

| Change | Source to edit | Checks or generated dependants |
|---|---|---|
| Vehicle measurement or geometry | `vehicle_params.yaml`, `components.yaml`, or `cad_parts.csv` | `gen_model_sdf.py`, `gen_docs.py`, baselines if behaviour changes |
| Control tuning | `control_gains.yaml` | scenarios and deliberate baseline review |
| Flight behaviour | `gnc/` | unit tests, scenarios, trace, and credibility claims |
| Analytic physics | `plant/` | scenarios, baseline, analytic/Gazebo comparison |
| Gazebo model or transport | generator, SDF inputs, `hal/`, or `harness/gz.py` | model check, servo tests, headless hover |
| ROS wrapper | `nodes/` or `launch/` | `colcon build` and the affected launch path |

Never hand-edit `gazebo/models/tvc_vehicle/model.sdf` or the generated regions
between `EMIT` markers in the numbered docs. Do not add a second controller,
copy a measured constant into code, or put ROS/Gazebo dependencies in `gnc/`.

## Portable merge gate

```bash
pip install -r requirements.txt
python -m pytest tests/ -q
python tools/gen_model_sdf.py --check
python tools/gen_docs.py --check
python tvc.py golden --check
python tvc.py validate --verbose
```

For ROS or Gazebo changes, also follow [docs/DEVCONTAINER.md](docs/DEVCONTAINER.md)
and run the relevant commands in [docs/6-RUNNING.md](docs/6-RUNNING.md). If that
environment is unavailable, state exactly which integration check was not run.

## Review checklist

- Behaviour changes have a focused regression test.
- Frames, signs, units, and actuator limits still match `docs/4-CONVENTIONS.md`.
- Generated files and frozen baselines are unchanged or deliberately updated.
- `docs/7-CREDIBILITY.md` still distinguishes verification from flight validation.
- New limitations and unresolved measurements are visible in the credibility
  record and roadmap.
- No generated run logs, caches, local settings, or build products are committed.

