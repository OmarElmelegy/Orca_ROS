# Orca ROV Project: Testing Guide

This guide provides detailed instructions for testing different components of the Orca ROV system.

## Prerequisites

- Docker container is running (see [WINDOWS_SETUP.md](WINDOWS_SETUP.md))
- Shell scripts are executable (`chmod +x *.sh`)
- Configuration files are executable (`./make_cfg_executable.sh`)
- Python scripts are executable (`./make_scripts_executable.sh`)
- ROS workspace is built (`./build_workspace.sh`)
- X Server is running for GUI applications

## Quick Test

To run a basic test of all components:

```bash
./test-all.sh
```

This script will check:
- If ROS is working
- If the URDF model can be parsed
- If the launch files are valid
- If the control system can be launched
- If the perception system can be launched

## Testing Individual Components

### 1. Testing the URDF Model

#### Visualize in RViz

```bash
roslaunch orca_description display.launch
```

This will open RViz with the Orca ROV model loaded. You can:
- View the 3D model from different angles
- Use the joint_state_publisher_gui to manipulate the grippers
- View the TF frames
- View simulated camera feeds

#### Check URDF Validity

```bash
check_urdf src/orca_description/urdf/orca.urdf.xacro
```

### 2. Testing the Control System

#### Launch the Control System

```bash
roslaunch orca_control control.launch
```

#### Test Depth Controller

```bash
# Set depth setpoint to 1 meter
rostopic pub /depth_setpoint std_msgs/Float64 "data: 1.0" -1

# Monitor the desired wrench
rostopic echo /desired_wrench
```

#### Test Heading Controller

```bash
# Set heading setpoint to 45 degrees (0.785 radians)
rostopic pub /heading_setpoint std_msgs/Float64 "data: 0.785" -1

# Monitor the desired wrench
rostopic echo /desired_wrench
```

#### Test Gripper Control

```bash
# Open left gripper
rostopic pub /left_gripper/command std_msgs/Bool "data: true" -1

# Close left gripper
rostopic pub /left_gripper/command std_msgs/Bool "data: false" -1
```

#### Test Dynamic Reconfigure

```bash
# Launch the dynamic reconfigure GUI
rosrun rqt_reconfigure rqt_reconfigure
```

This will open a GUI where you can adjust the thruster control parameters.

### 3. Testing the Hardware Interface

#### Launch the Hardware Interface

```bash
roslaunch orca_hw_interface hardware.launch
```

#### Test Pressure Sensor (Simulated)

```bash
# Monitor pressure data
rostopic echo /pressure/data

# Monitor depth
rostopic echo /depth
```

#### Test Camera Feeds

```bash
# View camera feeds in RViz or rqt_image_view
rosrun rqt_image_view rqt_image_view
```

### 4. Testing the Perception System

#### Launch the Perception System

```bash
roslaunch orca_perception perception.launch
```

#### Test Object Detection

```bash
# Monitor detected objects
rostopic echo /detected_object

# View detection image
rosrun rqt_image_view rqt_image_view /detection_image
```

#### Test Length Measurement

```bash
# Monitor object length
rostopic echo /object_length

# View point clouds in RViz
rosrun rviz rviz
# Add a PointCloud2 display and select the /filtered_cloud or /segmented_cloud topic
```

### 5. Testing Missions

#### Water Sampling Mission

```bash
roslaunch orca_bringup water_sampling_mission.launch
```

#### Object Retrieval Mission

```bash
roslaunch orca_bringup object_retrieval_mission.launch
```

#### Visualize Mission State Machine

```bash
rosrun smach_viewer smach_viewer.py
```

### 6. Testing the Complete System

#### Launch the Complete System

```bash
roslaunch orca_bringup orca_complete.launch
```

#### Launch the Simulation

```bash
roslaunch orca_bringup orca_simulation.launch
```

## Monitoring and Debugging

### List All Topics

```bash
rostopic list
```

### Monitor a Topic

```bash
rostopic echo /topic_name
```

### List All Nodes

```bash
rosnode list
```

### Get Node Information

```bash
rosnode info /node_name
```

### Visualize the Node Graph

```bash
rosrun rqt_graph rqt_graph
```

### View Log Messages

```bash
rosrun rqt_console rqt_console
```

## Common Issues and Solutions

### Build Issues

1. **Missing configuration files**:
   - Make sure to run `./make_cfg_executable.sh` before building the workspace
   - This will make all configuration files executable

2. **Missing dependencies**:
   - If you encounter missing dependencies, you can install them inside the container:
     ```bash
     apt-get update
     apt-get install ros-noetic-PACKAGE-NAME
     ```

### RViz Crashes (Segmentation Fault)

1. **Updated Docker Run Script**:
   - The `docker-run-gui.bat` script has been updated with additional environment variables to improve X11 forwarding
   - These include `LIBGL_ALWAYS_SOFTWARE=1`, `MESA_GL_VERSION_OVERRIDE=3.3`, and `MESA_GLSL_VERSION_OVERRIDE=330`

2. **Alternative Testing Methods**:
   - Use the `test-model-only.sh` script to test the ROV model without RViz
   - Use the `test-rviz-simple.sh` script to test RViz with a simple configuration

3. **Incremental Testing**:
   - First test without RViz
   - Then test RViz with a simple configuration
   - Finally test RViz with the ROV model

### RViz or Gazebo Not Appearing

1. Check if X Server is running with "Disable access control" checked
2. Verify DISPLAY environment variable:
   ```bash
   echo $DISPLAY
   # Should be host.docker.internal:0.0
   ```
3. Try setting it explicitly:
   ```bash
   export DISPLAY=host.docker.internal:0.0
   ```

### ROS Nodes Not Communicating

1. Check if ROS master is running:
   ```bash
   rostopic list
   ```
2. Check ROS_MASTER_URI:
   ```bash
   echo $ROS_MASTER_URI
   # Should be http://localhost:11311
   ```

### Missing Dependencies

If you encounter missing dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-PACKAGE-NAME
```

### Permission Issues

If you encounter permission issues with devices:
```bash
# Inside the container
sudo chmod a+rw /dev/ttyACM0  # For serial devices
sudo chmod a+rw /dev/video0   # For cameras
```
