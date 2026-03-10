# AI-Driven UAV System for Disaster Response

This repository contains a fully containerized, hardware-accelerated autonomous UAV software stack designed for real-time disaster response and target detection.

The architecture bridges high-level artificial intelligence (OpenCV, YOLO) with low-level aerodynamic physics and flight control. To ensure exact reproducibility across any Linux or Windows (WSL2) machine, the entire environment—including custom 3D terrain models, the LiDAR/Depth-equipped drone, and the complete ROS 2 ecosystem—is packaged into a single NVIDIA-accelerated Docker container.

## System Architecture: How the Components Work

When the container launches, a 5-pane `tmux` dashboard automatically initializes the following subsystems concurrently:

1. **Gazebo (The Physics Engine):** Simulates the custom 3D disaster terrain and calculates real-world physics for the drone. It natively generates the 360° LiDAR point clouds and RGB-D (Depth) camera matrices.
2. **ArduPilot SITL (The Flight Controller):** The "brain" of the drone. It receives the simulated physics data, runs complex PID stabilization loops, and calculates exact motor RPMs to keep the quadcopter airborne.
3. **MAVROS (The ROS 2 Bridge):** Translates high-level ROS 2 commands from the Python AI node into low-level MAVLink serial signals that ArduPilot can understand.
4. **ROS-GZ Bridge (The Sensor Pipeline):** Directly intercepts the raw LiDAR and Depth Camera byte arrays from Gazebo and exposes them to the ROS 2 network, bypassing standard flight controller bandwidth limits.
5. **AI Vision Controller (The Edge Logic):** A multi-threaded Python node that subscribes to the sensor bridges, runs computer vision inference to project targets into absolute global coordinates (GPS), and hosts an asynchronous Flask server for Ground Control commands.

---

## Build Instructions

**Prerequisites:**

* Docker installed and running.
* [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed (for GPU hardware acceleration).
* Linux host or Windows 11 with WSL2 (WSLg enabled).

Build the Docker image using BuildKit caching to dramatically speed up the installation of heavy machine learning libraries (PyTorch, OpenCV):

```bash
DOCKER_BUILDKIT=1 docker build -t disaster-droid-sim .

```

---

## Run Instructions

Before launching the container, you must grant Docker permission to render graphical windows on your host machine's X11 server:

```bash
xhost +local:root

```

Launch the simulation dashboard with full GPU passthrough and host networking:

```bash
docker run -it --rm \
  --gpus all \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="__NV_PRIME_RENDER_OFFLOAD=1" \
  --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  disaster-droid-sim

```

---

## IMPORTANT: Pre-Flight Safety Check

When the 5-pane dashboard boots up, the simulated flight controller requires time to calibrate its virtual sensors and lock onto the simulated GPS satellites.

**DO NOT** send a takeoff command immediately.

Watch the ArduPilot terminal (Top-Right pane). You must wait until you see these two exact lines appear in the console before initiating flight:

* `EKF3 IMU0 is using GPS`
* `EKF3 IMU1 is using GPS`

Once those lines appear, the Extended Kalman Filters (EKF) are fully initialized, and the drone has absolute spatial awareness.

---

## Ground Control: Command Line Interface (CLI)

The Python AI Vision Controller (Bottom-Right pane) includes an interactive command-line prompt. Once the EKF is initialized, you can manually maneuver the drone by typing the following commands directly into the terminal:

* `takeoff` : Arms the motors and ascends to the default altitude (15 meters).
* `go <latitude> <longitude> <raidus>` : Commands the drone to fly to the specified absolute global coordinates once it reaches the destination starts hovering the area covered by given radius with the coordinates as center of the circle.
* `stop` : Instantly halts the drone's current trajectory and forces it into a stable hover.
* `speed <value>` : Adjusts the drone's horizontal flight speed (in meters per second).
* `rtl` : Triggers the Return-To-Launch failsafe, bringing the drone back to its original spawn coordinates and landing.
* `land` : Commands the drone to descend and land exactly at its current position.

---

## Ground Control: REST API (`curl` commands)

The container also runs a Flask server mapped to your host machine's `localhost` on port 5000, allowing you to trigger commands remotely or from a frontend dashboard.

**System Status (GET):**
Check the drone's connection state, current coordinates, and flight mode.

```bash
curl http://127.0.0.1:5000/api/status

```

**Flight Commands (POST):**
Note: State-changing commands require the `-X POST` flag.

```bash
# Initiate takeoff
curl -X POST http://127.0.0.1:5000/api/takeoff

# Halt current movement and hover
curl -X POST http://127.0.0.1:5000/api/stop

# Return to launch point
curl -X POST http://127.0.0.1:5000/api/rtl

# Land at current location
curl -X POST http://127.0.0.1:5000/api/land

```

**Navigate to Coordinate (POST):**
Pass a JSON payload with the target coordinates.

```bash
curl -X POST http://127.0.0.1:5000/api/go \
     -H "Content-Type: application/json" \
     -d '{"lat": 33.643500, "lon": 72.992300, "radius": 15.0}'

```

**Set Speed (POST):**
Pass a JSON payload to set the speed in m/s.

```bash
curl -X POST http://127.0.0.1:5000/api/speed \
     -H "Content-Type: application/json" \
     -d '{"speed": 5.0}'

```

---

