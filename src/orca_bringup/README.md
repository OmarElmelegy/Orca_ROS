# Orca ROV Bringup Package

This package contains launch files for starting the Orca ROV system.

## Overview

The Orca ROV bringup package provides launch files for:
- Starting the core ROV systems
- Starting the complete ROV system
- Running the ROV in simulation
- Executing specific missions

## Launch Files

### orca_core.launch

Launches the core ROV systems:
- Robot description (URDF)
- Hardware interface
- Control system

```bash
roslaunch orca_bringup orca_core.launch
```

### orca_complete.launch

Launches the complete ROV system:
- Core ROV systems
- Perception system
- Visualization (RViz)

```bash
roslaunch orca_bringup orca_complete.launch
```

### orca_simulation.launch

Launches the ROV in simulation:
- Gazebo simulation
- Control system
- Perception system
- Visualization (RViz)

```bash
roslaunch orca_bringup orca_simulation.launch
```

### water_sampling_mission.launch

Launches the water sampling mission:
- Core ROV systems
- Water sampling mission

```bash
roslaunch orca_bringup water_sampling_mission.launch
```

### object_retrieval_mission.launch

Launches the object retrieval mission:
- Core ROV systems
- Perception system
- Object retrieval mission

```bash
roslaunch orca_bringup object_retrieval_mission.launch
```

## Usage

### Real Hardware

To start the ROV on real hardware:

```bash
# Start the core ROV systems
roslaunch orca_bringup orca_core.launch

# Start the complete ROV system
roslaunch orca_bringup orca_complete.launch

# Start a specific mission
roslaunch orca_bringup water_sampling_mission.launch
roslaunch orca_bringup object_retrieval_mission.launch
```

### Simulation

To start the ROV in simulation:

```bash
roslaunch orca_bringup orca_simulation.launch
```

## System Architecture

The Orca ROV system consists of the following components:

1. **orca_description**: URDF model and meshes
2. **orca_hw_interface**: Hardware drivers and interfaces
3. **orca_control**: Control algorithms and thruster allocation
4. **orca_perception**: Computer vision and sensor processing
5. **orca_missions**: Mission planning and execution

The bringup package provides launch files to start these components in different configurations.

## Troubleshooting

### ROS Master Issues

If you encounter issues with the ROS master:
```bash
# Check if the ROS master is running
rostopic list

# Restart the ROS master
killall -9 rosmaster
roscore
```

### Launch File Issues

If a launch file fails to start:
```bash
# Run with verbose output
roslaunch --verbose orca_bringup orca_core.launch
```

### Hardware Connection Issues

If hardware connections fail:
```bash
# Check USB devices
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
ls -l /dev/video*

# Check permissions
sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/video0
```
