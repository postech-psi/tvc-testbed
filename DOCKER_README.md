# Docker Dev Environment — TVC VTVL UGRP

This gets you a development environment — ROS2 Jazzy + Gazebo Harmonic + the PX4 bridge tool — that is identical on every machine it runs on, whether that's your laptop, a teammate's, or a lab workstation. That sameness is the entire point of using Docker here: "it works on my machine" stops being a possible excuse, because everyone's machine is running the same container.

## Concepts, briefly (skip if you already know Docker)

Three words come up constantly and are worth being precise about:

- **Image** — a built, read-only template (think: a class, not an instance). Defined by a `Dockerfile`. Building an image doesn't run anything; it just produces the template.
- **Container** — a running instance of an image (the instance of that class). You can start, stop, and delete containers freely without touching the image they came from.
- **Rebuild** — re-running the Dockerfile's instructions to produce a new image, needed whenever the Dockerfile itself changes. Editing your project's code does *not* require a rebuild — only editing the Dockerfile does.

A **Dev Container** (what VS Code's "Reopen in Container" button uses) is a further convention on top of plain Docker: a `.devcontainer/devcontainer.json` file tells VS Code which Dockerfile to build, which folder to mount your code into, and which editor extensions to install *inside* the container. The effect is that VS Code's terminal, IntelliSense, and debugger all operate inside the container, while you keep editing files as if they were local.

## Quick Start

**1. Install two things, once:**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) — during install, choose "install for this user" (no admin rights needed)
- [VS Code](https://code.visualstudio.com/) + its **Dev Containers** extension (`Ctrl+Shift+X` → search "Dev Containers" → Install)

**2. Open the project:**
```bash
cd tvc-testbed
code .
```

**3. Click "Reopen in Container"** when VS Code prompts you (a notification appears in the bottom-right corner).

What happens next, concretely: VS Code reads `.devcontainer/devcontainer.json`, which points at the `Dockerfile`; Docker builds an image from it (downloading each tool listed there); then VS Code starts a container from that image and reopens itself inside it. First time, this takes roughly 10 minutes, because every layer has to be downloaded and built. After that, Docker caches the result — reopening is a matter of seconds, and even a full rebuild after a small Dockerfile edit only redoes the layers after your change.

**4. Verify it worked.** Open a terminal inside VS Code (`` Ctrl+` ``) — this terminal is running inside the container, not on your host machine — and run:
```bash
ros2 --version      # → ROS 2 release 'jazzy'
gz sim --version    # → Gazebo Sim, version 8.x
```

If both print a version, the environment is ready. From here, edit code normally; files are synced live between your machine and the container (see "workspaceMount" in `devcontainer.json` — it's a bind mount, meaning the container is looking at the exact same files on disk as your editor, not a copy).

## What's inside, and why

| Tool | What it generally does | Why this project needs it | Phase |
|---|---|---|---|
| ROS2 Jazzy | Middleware for message-passing between processes ("nodes") over "topics" | The nodes you'll write for the vehicle's control loop communicate this way | 3 |
| Gazebo Harmonic | Physics simulator | Lets you test control code against simulated vehicle dynamics before real hardware exists (SIL: Simulation-In-the-Loop) | 4 |
| Micro-XRCE-DDS-Agent | Protocol bridge | Translates between ROS2's messaging format and PX4's own (uXRCE-DDS), so a ROS2 node can command a PX4 flight controller | 5 |
| numpy / scipy / matplotlib | Numerical computing / plotting | Same stack `tvc_physics.py` already uses, so results are identical whether run on your host or in this container | all |

Nothing here targets a Raspberry Pi or real flight hardware yet (Phase 6) — that will be a separate, smaller image built specifically for deployment, once there's an actual Pixhawk to talk to. Building that now would be maintaining a second thing with no way yet to test it.

**Deliberately excluded:** the ARM cross-compiler toolchain (`arm-none-eabi-gcc`). That toolchain compiles PX4 *firmware* for the Pixhawk's own microcontroller — a different target than the plain x86_64 build used by PX4 SITL (the simulated version used in Phases 4–5, which builds with the same compiler as everything else here). If a later phase needs to compile custom firmware, that's a one-line addition then, not a reason to carry it now.

## Project layout

```
Dockerfile             the image definition — see inline comments for what each instruction does
.devcontainer/          tells VS Code how to build and open the container
.dockerignore           files Docker should not copy into the build context (build artifacts, .git, etc.)
```

The `.dockerignore` file matters for a reason worth understanding generally: every file in your project folder (except what's excluded here) gets sent to the Docker build process as the "build context," even for files no instruction ever references. On a large repo with build artifacts or a `.git` history, that can make builds slow for no benefit — hence excluding them.

## Adding a library later

The general shape of adding anything to a Docker image is: edit the Dockerfile, then rebuild. Two common cases:

**A system/ROS2 package** (installed via `apt`) — add it to the relevant `apt-get install` block:
```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
```

**A Python package** (installed via `pip`) — add it to the `pip3 install` line:
```dockerfile
RUN pip3 install --break-system-packages --no-cache-dir \
    numpy scipy matplotlib pyyaml jinja2 empy opencv-python
```

Then in VS Code: `Ctrl+Shift+P` → **"Dev Containers: Rebuild Container"**. Docker only redoes the layer you changed and everything after it — layers before your edit are reused from cache — so this is usually 1–2 minutes, not a full 10-minute rebuild.

**Testing something before committing to it:** you can also install a package directly inside a running container (`sudo apt-get install ...` or `pip3 install --break-system-packages ...`), without touching the Dockerfile at all. It works immediately but disappears the next time the container is rebuilt — useful for trying something out before deciding it's worth adding permanently.

## Troubleshooting

| Problem | What's likely happening | Fix |
|---|---|---|
| "Docker daemon not running" | Docker Desktop isn't started, or hasn't finished starting | Launch Docker Desktop and wait for the whale icon to stop animating |
| Build fails partway through | Usually a network hiccup mid-download | Click "Reopen in Container" again — Docker resumes from the last successfully-built layer, it doesn't start over |
| `ros2: command not found` | You're running a command on your host machine, not inside the container | Check the bottom-left corner of VS Code — it should show the container name; if it shows your local machine, you're not connected |
| Edited the Dockerfile but nothing changed | Editing the Dockerfile only takes effect on the next build | `Ctrl+Shift+P` → "Dev Containers: Rebuild Container" |
| Gazebo's GUI window doesn't appear | Expected on Windows/Mac — the simulator runs headless there by default | Use `ros2 topic echo <topic-name>` to inspect data instead of relying on the window |
| **Windows**: "Container failed to start" (WSLg socket error) | VS Code's Dev Containers extension tries to forward your Wayland socket from WSL2 into the container so Linux GUI apps can display. WSL2 has a known limitation where Unix domain sockets don't survive the `\\wsl.localhost` network bridge, causing the mount to fail. | `Ctrl+Shift+P` → "Preferences: Open User Settings (JSON)" → add `"dev.containers.mountWaylandSocket": false` → save → rebuild the container. This disables GUI socket forwarding (not needed anyway, since Gazebo runs headless on Windows). |

## For teammates joining later

Same Quick Start steps above apply to everyone. Since the image only needs to be built once per machine and Docker caches the result afterward, a second person's first build is also fast (2–3 minutes) — the slow first build only happens once, on whichever machine builds this particular image for the very first time.
