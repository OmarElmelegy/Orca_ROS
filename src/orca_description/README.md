# Orca ROV Description Package

This package contains the URDF model and meshes for the Orca ROV.

## Overview

The Orca ROV description package defines the physical structure of the ROV, including:
- Main body dimensions and properties
- Thruster positions and orientations
- Camera placements
- Gripper configurations
- Sensor locations

## Usage

### Visualizing the ROV in RViz

```bash
roslaunch orca_description display.launch
```

This will start RViz with the ROV model loaded. You can use the joint_state_publisher_gui to manipulate the grippers.

### Simulating the ROV in Gazebo

```bash
roslaunch orca_description gazebo.launch
```

This will start Gazebo with the ROV model loaded in an underwater environment.

## Package Structure

- `urdf/`: URDF and XACRO files defining the ROV model
  - `orca.urdf.xacro`: Main ROV model
  - `materials.xacro`: Material definitions
  - `thrusters.xacro`: Thruster macros
  - `sensors.xacro`: Sensor macros
  - `grippers.xacro`: Gripper macros
- `meshes/`: 3D mesh files for visualization
- `launch/`: Launch files
  - `display.launch`: Launch file for RViz visualization
  - `gazebo.launch`: Launch file for Gazebo simulation
- `rviz/`: RViz configuration files
- `worlds/`: Gazebo world files
  - `underwater.world`: Underwater environment for simulation

## Model Details

The Orca ROV model includes:
- 8x Blue Robotics T200 thrusters in a vectored configuration
- 2x Newton grippers
- Intel RealSense D415 depth camera
- Hikvision 1080p front camera
- Generic rear camera
- MS5837-30BA pressure sensor
- IMU (representing the Pixhawk)

## Coordinate Frames

The model uses the following coordinate frames:
- `base_link`: The center of the ROV body
- `thruster_*_link`: Thruster frames
- `*_camera_link`: Camera frames
- `*_camera_optical_frame`: Camera optical frames
- `gripper_*_link`: Gripper frames
- `pressure_link`: Pressure sensor frame
- `imu_link`: IMU frame
