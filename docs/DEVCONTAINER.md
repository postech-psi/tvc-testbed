# The development container

**You only need this for the ROS 2 and Gazebo pipelines.** Everything else —
`tvc.py validate`, the GUI, the 3D viewer, the whole test suite — runs on a plain
host with `pip install -r requirements.txt`. If you are here to read or change
the flight code, you do not need Docker at all.

The container provides ROS 2 Jazzy, Gazebo Harmonic and the PX4 uXRCE-DDS bridge,
identical on every machine. That sameness is the entire point: *"it works on my
machine"* stops being a possible explanation.

---

## Quick start

**1. Install two things, once.**

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) — during
  install choose "install for this user" (no admin rights needed)
- [VS Code](https://code.visualstudio.com/) and its **Dev Containers**
  extension (`Ctrl+Shift+X` → search "Dev Containers")

**2. Open the project and click "Reopen in Container"** when VS Code prompts.

VS Code reads `.devcontainer/devcontainer.json`, which points at the
`Dockerfile`; Docker builds an image; VS Code starts a container and reopens
itself inside it. First time this takes roughly ten minutes because every layer
is downloaded and built. After that it is seconds, and even a full rebuild after
a small Dockerfile edit only redoes the layers after the change.

**3. Verify.** Open a terminal inside VS Code (`` Ctrl+` ``) — this terminal runs
*inside* the container:

```bash
ros2 --version      # -> ROS 2 release 'jazzy'
gz sim --version    # -> Gazebo Sim, version 8.x
```

**4. Build the workspace.** This has **never been done**; expect friction.

```bash
colcon build --packages-select tvc_msgs
source install/setup.bash
colcon build --packages-select tvc_control
source install/setup.bash
```

`source install/setup.bash` must be re-run in every *new* terminal — each
terminal is a separate shell.

**5. Fly it.**

```bash
bash gazebo/run_hover.sh --duration 30        # no ROS needed
ros2 launch tvc_control gazebo.launch.py gui:=false
```

Full detail: [6-RUNNING.md](6-RUNNING.md).

---

## Three words worth being precise about

- **Image** — a built, read-only template. Defined by the `Dockerfile`. Building
  one runs nothing; it produces the template.
- **Container** — a running instance of an image. Start, stop and delete them
  freely without touching the image.
- **Rebuild** — re-running the Dockerfile's instructions. Needed only when the
  *Dockerfile* changes. **Editing project code never requires a rebuild** —
  `/workspace` is a bind mount, so the container sees the same files on disk your
  editor does.

---

## What is inside, and why

| tool | why this project needs it |
|---|---|
| ROS 2 Jazzy | the node/topic middleware the shipping architecture uses |
| Gazebo Harmonic | the physics engine for software-in-the-loop testing |
| Micro-XRCE-DDS-Agent | translates between ROS 2's messaging and PX4's uXRCE-DDS, so a ROS 2 node can command a Pixhawk |
| numpy / scipy / matplotlib / pyyaml / pillow | the same stack the host-side tools use, so results are identical either way |

**Deliberately excluded: the ARM cross-compiler** (`arm-none-eabi-gcc`). That
toolchain compiles PX4 *firmware* for the Pixhawk's microcontroller — a different
target from the x86_64 build PX4 SITL uses. If a later phase needs custom
firmware that is a one-line addition then, not a reason to carry it now.

Nothing here targets a Raspberry Pi or real flight hardware; that will be a
separate, smaller deployment image once there is a Pixhawk to talk to.

---

## Adding a library

Edit the Dockerfile, then rebuild (`Ctrl+Shift+P` → "Dev Containers: Rebuild
Container"). Docker reuses every layer before your edit, so this is usually one
to two minutes.

```dockerfile
# a system or ROS 2 package
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# a Python package
RUN pip3 install --break-system-packages --no-cache-dir \
    numpy scipy matplotlib pyyaml pillow pytest
```

To try something before committing to it, install it directly in the running
container (`sudo apt-get install …`). It works immediately and disappears on the
next rebuild.

---

## Troubleshooting

| problem | what is happening | fix |
|---|---|---|
| "Docker daemon not running" | Docker Desktop is not started | launch it and wait for the whale to stop animating |
| build fails partway | usually a network hiccup mid-download | click "Reopen in Container" again; Docker resumes from the last good layer |
| `ros2: command not found` | you are on the host, not in the container | check the bottom-left corner of VS Code for the container name |
| edited the Dockerfile, nothing changed | it only takes effect on the next build | "Dev Containers: Rebuild Container" |
| Gazebo's window does not appear | expected on Windows and macOS — the container has no display | run headless: `gui:=false`, and inspect with `ros2 topic echo` |
| **Windows:** "Container failed to start" (WSLg socket error) | the extension forwards your Wayland socket from WSL2 into the container; Unix domain sockets do not survive the `\\wsl.localhost` bridge | add `"dev.containers.mountWaylandSocket": false` to your VS Code user settings, then rebuild. Not needed anyway — Gazebo runs headless there. |
| Source Control panel empty, or "dubious ownership" | git refuses to operate on a repo whose file ownership does not match the current user | `git config --global --add safe.directory /workspace`, then reload the window |

---

## Git identity

The container does **not** inherit your host's git identity. Set it once per
environment you actually commit from:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

Most people commit from the host and use the container terminal only for ROS 2
and Gazebo.
