# ==============================================================================
# TVC VTVL UGRP — Development Environment
# ==============================================================================
# New to Docker? Read on — every instruction below is explained in general
# terms first, then tied to why this project needs it. If you just want to
# get running, see DOCKER_README.md instead.
#
# The one-sentence version of what a Dockerfile is: a recipe for building a
# filesystem image, executed top to bottom, where each instruction adds one
# layer on top of the last. "Build the image" replays this recipe once;
# "run a container" boots a live instance of the resulting image.
#
# Scoping a development image: include the toolchain your current work
# actually exercises, not everything the target hardware could ever need.
# Robotics and embedded projects generally split into two categories of
# tooling that are easy to conflate but serve different purposes:
#   - Host-side tooling — middleware, simulators, and bridges that compile
#     and run on the same machine architecture you're developing on.
#   - Cross-compilation toolchains — compilers that run on your dev
#     machine but produce binaries for a *different* target CPU (e.g. a
#     flight controller's own microcontroller). These only matter once
#     you're building firmware to flash onto physical hardware.
#
# This image covers the first category: ROS2 (middleware for message-
# passing between processes), a physics simulator (Gazebo), and a bridge
# tool that lets ROS2 talk to a flight controller's own protocol. That
# combination is enough to write and test control code entirely in
# simulation before any hardware exists.
#
# Deliberately NOT included: an ARM cross-compiler. PX4 (the flight-
# control firmware this project bridges to) has two distinct build
# targets that are easy to confuse: SITL (Software-In-The-Loop), an
# ordinary x86_64 program used for simulation, and the real firmware
# image meant to be flashed onto the flight controller's own
# microcontroller. Only the second needs a cross-compiler. Since SITL
# builds with the same compiler as everything else in this image, there's
# no reason to carry a cross-compilation toolchain until it's time to
# actually build and flash custom firmware — add arm-none-eabi-gcc then.
# ==============================================================================

# ------------------------------------------------------------------------------
# FROM <image> — every Dockerfile starts here. It picks the base layer
# everything else builds on top of, pulled from a registry (Docker Hub by
# default). You're never building an OS from scratch; you're starting from
# someone else's published image and adding to it.
#
# `ros:jazzy` is the official ROS2 image, maintained by Open Robotics. It's
# Ubuntu 24.04 with ROS2 Jazzy already installed and its apt package
# repository already configured — using it means we skip re-solving a
# problem (installing ROS2 cleanly) that upstream has already solved.
# General rule of thumb: prefer an official base image for your main
# framework over installing it yourself on a generic ubuntu:24.04 base.
# ------------------------------------------------------------------------------
FROM ros:jazzy

# ------------------------------------------------------------------------------
# ENV <key>=<value> — sets an environment variable that persists into every
# later instruction in this build AND into every container run from the
# final image. Contrast with a shell export inside a RUN command, which
# only lives for that one instruction.
# ------------------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
# Tells apt-get to never wait for interactive prompts (timezone pickers,
# etc). Without this, a build can hang forever waiting for keyboard input
# that will never come, since there's no terminal attached during a build.
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8

# ------------------------------------------------------------------------------
# RUN <command> — executes a shell command at build time and commits the
# resulting filesystem change as a new layer. Layers are cached: if you
# change a line further down the file, Docker reuses every unchanged layer
# above it instead of redoing the work. This is why related installs are
# usually grouped into one RUN (fewer, larger layers that make sense as a
# unit) rather than split across many.
#
# The `&& apt-get clean && rm -rf /var/lib/apt/lists/*` at the end of an
# apt RUN is a standard convention, not project-specific: apt downloads a
# package index before installing anything, and that index is dead weight
# once the install is done. Deleting it in the *same* RUN (not a later one)
# matters, because each RUN is a new layer — deleting a file in a later
# layer hides it from `du` but the bytes are still stored in the earlier
# layer underneath.
# ------------------------------------------------------------------------------

# Build tools, rosdep, colcon — this project's core toolchain
# --------------------------------------------------------------
# build-essential/cmake/ninja: compile ROS2 packages (`colcon build` calls
#   these under the hood for any C++ nodes you add).
# rosdep: resolves "this package needs libfoo-dev" style dependencies from
#   a package.xml, the standard way ROS2 workspaces declare what they need.
# colcon: the standard ROS2 build tool (successor to catkin from ROS1).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git curl wget sudo \
    lsb-release ca-certificates \
    python3-pip python3-venv \
    python3-rosdep python3-colcon-common-extensions python3-vcstool \
    vim nano \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# rosdep needs a one-time initialization (fetches the index of known ROS
# package-to-system-package mappings) before `rosdep install` works anywhere
# in this container. `2>/dev/null || true` makes the init step a no-op if
# some future base image ever ships with it already initialized, instead of
# failing the whole build over something harmless.
RUN rosdep init 2>/dev/null || true && rosdep update

# ------------------------------------------------------------------------------
# Gazebo Harmonic — added via a *second* apt repository
#
# General pattern: not every package you need lives in your base image's
# default apt repository. Adding a new one is always the same three steps:
#   1. Fetch the repo's signing key (so apt can verify packages aren't
#      tampered with) and save it under /usr/share/keyrings/.
#   2. Write a .list file telling apt the repo's URL and which key signs it
#      (the "signed-by=" reference ties steps 1 and 2 together).
#   3. `apt-get update` again so apt reads the new repo's index, then
#      install from it like any other package.
#
# Project-specific part: Gazebo Harmonic is the simulator version that
# ROS2 Jazzy officially pairs with, used here for SIL (Software-In-the-Loop)
# testing — running the control nodes against simulated vehicle physics
# before any real hardware exists to test them on.
# ------------------------------------------------------------------------------
RUN mkdir -p /usr/share/keyrings && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Micro-XRCE-DDS-Agent — built from source, not apt
#
# Not everything you need ships as a package at all. When it doesn't, the
# fallback is the same as on a bare metal machine: clone the source, run its
# usual build (cmake/make here), `make install` to put the binary somewhere
# on PATH (/usr/local/bin by convention), then delete the source tree so it
# doesn't sit around bloating the image once you have the compiled result.
#
# Project-specific part: this tool is the actual bridge between a ROS2 node
# and PX4's own message protocol (uXRCE-DDS). It lets a ROS2 controller node
# talk to PX4 — first PX4 SITL (simulated), later a real Pixhawk over a
# serial/USB link — without either side needing to know the other's native
# message format.
# Pinned to v2.4.3, not v2.4.2: v2.4.2's own build script tries to fetch a
# Fast-DDS branch (2.12.x) that eProsima has since deleted upstream, so it
# fails a fresh build with "invalid reference: 2.12.x". v2.4.3 is a same-line
# patch release (not the v3.x major bump, which changes the wire protocol)
# that doesn't carry this broken reference — a version pin still needs to be
# revisited if the *pinned* version's own dependencies move out from under it.
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends libssl-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cd /tmp/xrce && mkdir build && cd build \
    && cmake .. && make -j"$(nproc)" && make install && ldconfig \
    && rm -rf /tmp/xrce

# ------------------------------------------------------------------------------
# Python packages
#
# `--break-system-packages`: as of Ubuntu 24.04 / Debian's PEP 668 policy,
# `pip install` refuses to touch the system Python by default, because on a
# normal machine that risks clobbering packages your OS depends on. Inside
# a container, there's no "OS's Python" to protect beyond this one image's
# purpose, so overriding the guard is the accepted practice here — it would
# NOT be the right call on your actual laptop's system Python.
#
# numpy/scipy/matplotlib: the same stack this project's simulator already
#   uses, so results are identical whether run on your host machine or in
#   this container.
# jinja2/empy/pyyaml: PX4's own build system (not this project's code)
#   depends on these when building PX4 from source — without them, that
#   build fails on a missing Python dependency, not a C++ one.
# ------------------------------------------------------------------------------
RUN pip3 install --break-system-packages --no-cache-dir \
    numpy scipy matplotlib pyyaml jinja2 empy

# ------------------------------------------------------------------------------
# Non-root user
#
# General practice: everything up to this point ran as root, because a
# fresh container's default user is root unless told otherwise. Root
# inside a container isn't as dangerous as root on your actual machine
# (it's still confined to the container), but it's still good hygiene to
# do your actual work as an unprivileged user — it catches file-permission
# bugs early instead of only ever masking them, and it matches how services
# run in most real deployments.
#
# Project-specific part: this also matches how this software will run on
# real deployment hardware later — not as root there either — so nothing
# about file permissions or ownership changes when this workflow moves
# from simulation onto physical hardware.
# ------------------------------------------------------------------------------
RUN useradd -m -s /bin/bash ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros \
    && mkdir -p /workspace && chown -R ros:ros /workspace

# USER <name> — every instruction from this point on (and every command
# run in a live container from this image) executes as this user instead
# of root. `sudo` is still available (configured above) for the occasional
# case you need it, e.g. installing something ad hoc while testing.
USER ros

# Runs once per user creation. `~/.bashrc` is read every time an
# interactive bash shell starts, so these exports are how "ROS2 is just
# available" every time you open a terminal in this container, instead of
# something you'd type by hand each session.
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=/workspace/models" >> ~/.bashrc

# WORKDIR <path> — sets the working directory for the remaining build
# instructions and, importantly, the directory a container starts in when
# it boots. This is where your project's code will be mounted (see
# .devcontainer/devcontainer.json's workspaceMount).
WORKDIR /workspace

# CMD <command> — the default command a container runs if you don't
# specify one. VS Code's Dev Containers extension overrides this anyway
# (it needs its own long-lived process to attach to), so this mainly
# matters if you ever run the image directly with `docker run -it ...`.
CMD ["/bin/bash"]
