# Orca ROV ROS Project

This repository contains the ROS (Robot Operating System) project for the Orca Remotely Operated Vehicle (ROV)

## Overview

The Orca ROV is designed for underwater exploration and tasks, featuring:
- 8x Blue Robotics T200 thrusters
- 2x Newton grippers
- Multiple cameras (RealSense D415, Hikvision 1080p, rear camera)
- Pixhawk 2.4.8 flight controller
- Raspberry Pi 4B onboard computer
- Various sensors (IMU, pressure, etc.)

## Project Structure

The project is organized into the following ROS packages:

- **orca_description**: URDF models and meshes for the ROV
- **orca_control**: Control algorithms and thruster allocation
- **orca_hw_interface**: Hardware drivers and interfaces
- **orca_perception**: Computer vision and sensor processing
- **orca_missions**: Mission planning and execution
- **orca_bringup**: Launch files for starting the system

## Getting Started

### Prerequisites

- Ubuntu 20.04 LTS (Focal Fossa)
- ROS Noetic Ninjemys
- Docker (for development container)
- VS Code with Remote - Containers extension (for development)

### Development Environment

This project uses a Docker container with ROS Noetic for development. To use it:

1. Install Docker and VS Code with the Remote - Containers extension
2. Open this folder in VS Code
3. When prompted, click "Reopen in Container"
4. VS Code will build the Docker image and start the container

## Building the Project

```bash
# Navigate to the workspace root
cd /workspaces/Orca_ws

# Build the workspace
catkin build

# Source the workspace
source devel/setup.bash
```

## Running the ROV

### Basic Launch

```bash
# Launch the core ROV nodes
roslaunch orca_bringup orca_core.launch

# Launch the complete ROV system
roslaunch orca_bringup orca_complete.launch
```

### Simulation

```bash
# Launch the ROV in simulation
roslaunch orca_bringup orca_simulation.launch
```

### Missions

```bash
# Launch the water sampling mission
roslaunch orca_bringup water_sampling_mission.launch

# Launch the object retrieval mission
roslaunch orca_bringup object_retrieval_mission.launch
```

## Hardware Setup

### Onboard Computer

The Raspberry Pi 4B should be set up with:
- Ubuntu 20.04 LTS
- ROS Noetic
- Required ROS packages

### Flight Controller

The Pixhawk 2.4.8 should be set up with:
- ArduSub firmware
- Connected to the Raspberry Pi via USB
- Configured for the 8-thruster vectored configuration

### Cameras

- RealSense D415: Connected via USB
- Hikvision 1080p: Connected via USB
- Rear camera: Connected via USB

### Pressure Sensor

The MS5837-30BA pressure sensor should be connected to the Raspberry Pi via I2C.

### Thrusters and Grippers

The T200 thrusters and Newton grippers are controlled via the Pixhawk's PWM outputs.

## Network Setup

The ROV uses an Ethernet tether for communication with the topside computer:

- ROV (Raspberry Pi): 192.168.2.2
- Topside computer: 192.168.2.1
- QGroundControl connects to the ROV via UDP on port 14550

## Documentation

Each package contains a README file with detailed information:

- [orca_description/README.md](src/orca_description/README.md)
- [orca_control/README.md](src/orca_control/README.md)
- [orca_hw_interface/README.md](src/orca_hw_interface/README.md)
- [orca_perception/README.md](src/orca_perception/README.md)
- [orca_missions/README.md](src/orca_missions/README.md)
- [orca_bringup/README.md](src/orca_bringup/README.md)

## License

This project is licensed under the MIT License - see the LICENSE file for details.
