# Linux development environment for ROS 2 Jazzy and Gazebo Harmonic.
FROM ros:jazzy
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl sudo lsb-release ca-certificates \
    python3-pip python3-tk python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic ros-jazzy-ros-gz-bridge ros-jazzy-actuator-msgs \
    && rm -rf /var/lib/apt/lists/*

# Use Ubuntu Python packages alongside ROS; pip must not replace apt-owned NumPy.
RUN apt-get -o Acquire::Retries=3 update --error-on=any \
    && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
    python3-numpy python3-scipy python3-matplotlib python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# ROS adds vendor command paths; retain the system Gazebo sim command as well.
ENV GZ_CONFIG_PATH=/usr/share/gz

RUN useradd -m -s /bin/bash ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros \
    && mkdir -p /workspace && chown ros:ros /workspace
USER ros
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
WORKDIR /workspace
CMD ["/bin/bash"]
