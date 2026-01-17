
# AutoPilot - ADAS Development Framework

A modular Advanced Driver Assistance System (ADAS) framework built on ROS Noetic and CARLA 0.9.13, featuring Automatic Emergency Braking (AEB) with hardware abstraction for Sim2Real migration.

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Configuration](#configuration)
- [Testing](#testing)
- [Development](#development)
- [License](#license)

## Features

- **Automatic Emergency Braking (AEB)**: TTC-based collision avoidance with configurable thresholds
- **LiDAR Perception**: Point cloud processing with DBSCAN clustering for obstacle detection
- **Safety Monitor**: ISO 26262 inspired watchdog with 100ms heartbeat monitoring
- **Hardware Abstraction Layer (HAL)**: Unified sensor interface for simulation and real vehicle
- **Modular Architecture**: Independent ROS packages for perception, decision, control, and safety

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         ADAS System                             │
├─────────────┬─────────────┬─────────────┬─────────────┬─────────┤
│ Perception  │  Decision   │   Safety    │   Control   │   HAL   │
├─────────────┼─────────────┼─────────────┼─────────────┼─────────┤
│ LiDAR Proc  │ TTC Calc    │ Safety Mon  │ Vehicle Ctrl│ Sensor  │
│ Obstacle    │ AEB FSM     │ Heartbeat   │ Brake Act   │ Bridge  │
│ Tracker     │             │ Watchdog    │             │         │
└─────────────┴─────────────┴─────────────┴─────────────┴─────────┘
                              │
                    ┌─────────┴─────────┐
                    │   CARLA / Real    │
                    │     Vehicle       │
                    └───────────────────┘
```

### ROS Packages

| Package | Description |
|---------|-------------|
| `adas_msgs` | Custom message and service definitions |
| `adas_perception` | LiDAR processing, obstacle detection and tracking |
| `adas_decision` | TTC calculation, AEB finite state machine |
| `adas_safety` | Safety monitor, heartbeat watchdog |
| `adas_control` | Vehicle controller, brake actuator |
| `adas_hal` | Hardware abstraction layer, sensor bridges |
| `adas_bringup` | Launch files, system configuration |

## Prerequisites

- Ubuntu 20.04 LTS
- ROS Noetic
- CARLA 0.9.13
- Python 3.8+
- PCL 1.10+

## Installation

### 1. Install ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full -y
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo rosdep init
rosdep update

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install CARLA 0.9.13

```bash
# Download from https://github.com/carla-simulator/carla/releases/tag/0.9.13/
mkdir ~/CARLA && tar -zxvf CARLA_0.9.13.tar.gz -C ~/CARLA

# Install dependencies
sudo apt install libomp5 libvulkan1 xdg-user-dirs python-is-python3 -y

# Install Python API
pip3 install --user pygame numpy
pip3 install ~/CARLA/PythonAPI/carla/dist/carla-0.9.13-cp38-cp38-linux_x86_64.whl
```

### 3. Build CARLA-ROS Bridge

```bash
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge/catkin_ws/src
git clone -b '0.9.13' --recursive https://github.com/carla-simulator/ros-bridge.git

sudo apt install ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs ros-noetic-tf2-sensor-msgs -y

cd ~/carla-ros-bridge/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

### 4. Build ADAS Workspace

```bash
cd ~/AutoPilot/catkin_ws

# Source carla-ros-bridge first (creates overlay workspace)
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash

# Install dependencies
sudo apt install ros-noetic-pcl-ros python3-sklearn -y
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make
```

### 5. Configure Environment

Add to `~/.bashrc`:

```bash
export CARLA_ROOT=~/CARLA
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.8-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla

# Source ADAS workspace (automatically includes carla-ros-bridge)
source ~/AutoPilot/catkin_ws/devel/setup.bash
```

## Quick Start

### Terminal 1: Start CARLA Server

```bash
cd $CARLA_ROOT && ./CarlaUE4.sh -quality-level=Low
```

### Terminal 2: Start CARLA-ROS Bridge

```bash
roslaunch carla_ros_bridge carla_ros_bridge.launch
```

### Terminal 3: Spawn Ego Vehicle

```bash
roslaunch carla_spawn_objects carla_spawn_objects.launch
```

### Terminal 4: Start ADAS System

```bash
roslaunch adas_bringup adas_full_sim.launch
```

### Terminal 5: Manual Control (for testing)

```bash
python3 $CARLA_ROOT/PythonAPI/examples/manual_control.py
```

## Usage

### Launch Options

```bash
# Full system with RViz visualization
roslaunch adas_bringup adas_full_sim.launch

# Without RViz (lower resource usage)
roslaunch adas_bringup adas_full_sim.launch enable_rviz:=false

# With camera detector enabled
roslaunch adas_bringup adas_full_sim.launch enable_camera:=true

# Debug mode (single stack)
roslaunch adas_bringup adas_debug.launch
```

### Monitor Topics

```bash
# AEB state and transitions
rostopic echo /adas/aeb_state

# Detected obstacles
rostopic echo /adas/obstacles

# Time-to-Collision information
rostopic echo /adas/ttc_info

# Safety system status
rostopic echo /adas/safety_status
```

### AEB Service

```bash
# Disable AEB
rosservice call /adas/set_aeb_mode "enable: false"

# Enable AEB
rosservice call /adas/set_aeb_mode "enable: true"
```

## Configuration

### AEB Thresholds

Edit `catkin_ws/src/adas_decision/config/aeb_config.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ttc_warning` | 2.5s | Enter WARNING state |
| `ttc_partial` | 1.5s | Enter PARTIAL_BRAKE state |
| `ttc_full` | 0.6s | Enter FULL_BRAKE state |
| `decel_partial_min` | 3.0 m/s² | Minimum partial braking |
| `decel_partial_max` | 6.0 m/s² | Maximum partial braking |
| `decel_full` | 10.0 m/s² | Full braking deceleration |

### LiDAR Processing

Edit `catkin_ws/src/adas_perception/config/lidar_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.15m | Downsampling resolution |
| `cluster_eps` | 0.8m | DBSCAN clustering radius |
| `cluster_min_samples` | 5 | Minimum points per cluster |
| `min_z` | -1.8m | Ground filtering threshold |
| `max_distance` | 50.0m | Maximum detection range |

### Safety Monitor

Edit `catkin_ws/src/adas_safety/config/safety_rules.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `watchdog_timeout_ms` | 100.0 | Heartbeat timeout (simulation) |
| `safe_deceleration` | 5.0 m/s² | Override deceleration |

## Testing

### AEB Collision Avoidance Test

1. Start the complete system (see Quick Start)
2. Use `manual_control.py` to drive toward a parked vehicle
3. Observe AEB state transitions in the terminal:
   - `INACTIVE -> WARNING` when TTC < 2.5s
   - `WARNING -> PARTIAL_BRAKE` when TTC < 1.5s
   - `PARTIAL_BRAKE -> FULL_BRAKE` when TTC < 0.6s

### Verify Data Flow

```bash
# Check LiDAR data
rostopic hz /adas/lidar

# Check obstacle detection
rostopic echo /adas/obstacles

# Check TTC calculation
rostopic echo /adas/ttc_info
```

## Development

### Project Structure

```
AutoPilot/
├── catkin_ws/
│   └── src/
│       ├── adas_msgs/          # Message definitions
│       ├── adas_perception/    # Perception stack
│       ├── adas_decision/      # Decision stack
│       ├── adas_safety/        # Safety monitor
│       ├── adas_control/       # Control stack
│       ├── adas_hal/           # Hardware abstraction
│       └── adas_bringup/       # Launch files
├── scenarios/                  # OpenSCENARIO test files
├── scripts/                    # Utility scripts
├── DEVLOG.md                   # Development log
└── readme.md
```

### Adding a New Module

1. Create a new ROS package in `catkin_ws/src/`
2. Add dependencies to `package.xml` and `CMakeLists.txt`
3. Include in appropriate launch file
4. Document in DEVLOG.md

### Sim2Real Migration

The HAL layer (`adas_hal`) provides abstraction for sensor data:

- **Simulation**: `sensor_bridge_carla.py` converts CARLA topics to unified `/adas/*` topics
- **Real Vehicle**: Implement `sensor_bridge_real.py` with actual sensor drivers

To switch modes, modify the launch file or use the `mode` parameter:

```bash
roslaunch adas_bringup adas_full_real.launch  # For real vehicle
```

## License

This project is for educational and research purposes.
