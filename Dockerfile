# syntax=docker/dockerfile:1

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install System Dependencies & Build Tools
RUN apt-get update && apt-get install -y \
    python3-pip git wget cmake lsb-release gnupg curl tmux \
    ros-jazzy-mavros ros-jazzy-mavros-extras \
    ros-jazzy-vision-msgs ros-jazzy-ros-gz \
    rapidjson-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    fonts-dejavu python3-pexpect \
    && rm -rf /var/lib/apt/lists/*

# Add the official Gazebo APT Repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install the missing Gazebo Harmonic development headers
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ArduPilot Gazebo Plugin into the container's system path
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git /ardupilot_gazebo && \
    cd /ardupilot_gazebo && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && make install

# Install ArduPilot SITL
RUN git clone https://github.com/ArduPilot/ardupilot.git /ardupilot && \
    cd /ardupilot && git submodule update --init --recursive

# Install lightweight drone & system dependencies first
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --default-timeout=1000 --retries=10 --break-system-packages --ignore-installed "numpy<2" flask MAVProxy pexpect future

# Install massive AI dependencies (PyTorch & Vision)
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --default-timeout=1000 --retries=10 --break-system-packages --ignore-installed ultralytics opencv-python

# Download and install the GeographicLib datasets required by MAVROS
RUN curl -s https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sed 's/sudo//g' | bash

# PRE-COMPILE ArduCopter so it boots instantly at runtime
RUN cd /ardupilot && \
    ./waf configure --board sitl && \
    ./waf copter

WORKDIR /workspace

# Copy your ENTIRE src folder (which now contains the AI code AND sim_assets!)
COPY src /workspace/src

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Copy the launch script and make it the entrypoint
COPY launch_docker.sh /launch_docker.sh
RUN chmod +x /launch_docker.sh

ENTRYPOINT ["/launch_docker.sh"]
