# TVC simulation

This project simulates an electric vehicle with two counter-rotating propellers and a two-axis thrust-vectoring gimbal. It is the simulation foundation for the POSTECH UGRP demonstrator, with a future **Raspberry Pi 5 + Pixhawk 6C** hardware system.

**Start with the GUI.** It uses the same controller and physics as the terminal command. The default run recovers from a small tilt while holding 2 m altitude and horizontal position.

## Run on Windows

From this folder, using Python 3.10 or newer:

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python tvc.py gui
```

For terminal operation, use that same Python executable:

```powershell
.venv\Scripts\python tvc.py sim
.venv\Scripts\python tvc.py sim --duration 10 --altitude 2.5
.venv\Scripts\python tvc.py plot out/simulation.csv out/flight.png
```

`sim` saves `out/simulation.csv` and `out/simulation.png`. Runs with the same output prefix replace those files; use `--output out/my-run` to keep a separate run. The GUI has Run, Reset, Save CSV, and Play CAD motion buttons. Its plot toolbar saves images. GUI edits last for that session.

## Run Gazebo through ROS

Use the Linux devcontainer, or a Linux environment with ROS 2 Jazzy and Gazebo Harmonic. In VS Code, choose **Dev Containers: Reopen in Container** with Docker Desktop running. Build once in the container:

The Dockerfile installs NumPy, SciPy, Matplotlib, and PyYAML through Ubuntu packages alongside ROS. The pip setup above is for your Windows virtual environment.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 tvc.py gazebo --headless
```

Stop with Ctrl+C, then run `python3 tvc.py plot out/gazebo.csv`. On Linux with a working graphical display, omit `--headless` for the Gazebo window. The Windows container does not configure display forwarding; the Python GUI runs directly on Windows.

Run `source install/setup.bash` in each new ROS terminal. Rebuild after changing package files or adding modules. Gazebo constructs its model from the current settings at launch; there is no CAD-inertia generation step.

## Where to look

| I want to… | Open this |
|---|---|
| Change mass, inertia, gimbal or motor values | [Vehicle settings](src/tvc_control/tvc_control/settings/vehicle.yaml) |
| Tune the controller | [Controller gains](src/tvc_control/tvc_control/settings/gains.yaml) |
| Understand a simulation step | [simulation.py](src/tvc_control/tvc_control/simulation.py) |
| Follow the control calculations | [controller.py](src/tvc_control/tvc_control/control/controller.py) |
| Understand ROS, Pixhawk, and every remaining file | [The guide](docs/GUIDE.md) |

The final CAD mesh is **appearance only**. Existing mass/inertia values and bench calibrations were retained; they are not automatically measurements of the final hardware. This currently demonstrates attitude/hover control, not a completed takeoff–descent–landing sequence or a hardware flight controller.

## Small check

```bash
python -m unittest discover -s tests
```

Only three short checks remain. Historical tools, extensive tests, reports, and previous layouts were saved outside this repo at `C:\Users\tae06\CODE\tvc-testbed-cleanup-20260920`. See the guide for recovery details.
