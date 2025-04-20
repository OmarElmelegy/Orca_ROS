# Orca ROV Project: Windows Quick Start Guide

This quick start guide will help you get the Orca ROV project running on Windows as quickly as possible.

## Prerequisites

- [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop)
- [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/)

## Step 1: Start the X Server

1. Launch VcXsrv (XLaunch)
2. Select "Multiple windows"
3. Display number: 0
4. Select "Start no client"
5. **IMPORTANT**: Check "Disable access control"
6. Click "Next" and then "Finish"

## Step 2: Build and Run the Docker Container

1. Open Command Prompt in the Orca_ws directory
2. Build the Docker image:
   ```
   docker-build.bat
   ```
3. Run the Docker container with GUI support:
   ```
   docker-run-gui.bat
   ```

## Step 3: Build the ROS Workspace

Inside the Docker container:
```bash
# Make all scripts executable
chmod +x *.sh

# Run the scripts to prepare the workspace
./make_scripts_executable.sh
./make_cfg_executable.sh

# Build the workspace
./build_workspace.sh
```

## Step 4: Test the System

### Visualize the ROV in RViz
```bash
roslaunch orca_description display.launch
```

### Run the ROV in Gazebo Simulation
```bash
roslaunch orca_bringup orca_simulation.launch
```

### Test a Mission
```bash
# Water sampling mission
roslaunch orca_bringup water_sampling_mission.launch

# Object retrieval mission
roslaunch orca_bringup object_retrieval_mission.launch
```

## Testing the System

### Testing Without RViz (Recommended for Initial Test)

If you're having issues with RViz, you can test the system without it:

```bash
# Test the ROV model without RViz
./test-model-only.sh
```

### Testing with Simple RViz Configuration

If you want to test RViz with a simple configuration:

```bash
# Test RViz with a simple configuration
./test-rviz-simple.sh

# Test RViz with a custom simple configuration
./test-rviz-custom.sh

# Test RViz with a minimal configuration
./test-rviz-minimal.sh

# Test RViz with software rendering explicitly enabled
./test-rviz-software.sh
```

## Common Issues

### RViz Crashes (Segmentation Fault)

- RViz may crash with a segmentation fault (exit code -11) due to X11 display issues
- The `docker-run-gui.bat` script has been updated with additional environment variables to improve X11 forwarding
- Try using the `test-model-only.sh` script to test the ROV model without RViz
- Try using the `test-rviz-simple.sh` script to test RViz with a simple configuration
- Try using the `test-rviz-custom.sh` script to test RViz with a custom simple configuration
- Try using the `test-rviz-minimal.sh` script to test RViz with a minimal configuration
- Try using the `test-rviz-software.sh` script to test RViz with software rendering explicitly enabled

### Docker Build Issues

- **Package not found errors**: The Dockerfile has been updated to use the correct ROS package names:
  - The `python3-smach` and `python3-smach-ros` packages have been replaced with `ros-noetic-executive-smach`, `ros-noetic-smach-ros`, and `ros-noetic-smach-viewer`
  - The `ros-noetic-v4l2-camera` package has been removed as it's not available in the standard ROS Noetic repositories
  - Added `libeigen3-dev` package for the Eigen library used by the thruster allocator

- **Network issues**: Make sure you have a stable internet connection during the build process.

- **Disk space issues**: Docker images can be large. Make sure you have at least 10GB of free disk space.

### Build Issues

- **Missing MS5837 implementation**: The CMakeLists.txt has been updated to compile the MS5837.cpp file and link it to the pressure_sensor_node. Also fixed a naming conflict with the system `read()` function by using the global scope operator (`::`).

- **Missing Eigen library**: The Dockerfile has been updated to include the libeigen3-dev package, and the CMakeLists.txt has been updated to find and include the Eigen library.

### Runtime Issues

- **No GUI appears**: Make sure VcXsrv is running with "Disable access control" checked.

- **Docker errors**: Ensure Docker Desktop is running.

- **Build errors**: Check that all dependencies are installed in the Docker container.

For more detailed instructions, see [WINDOWS_SETUP.md](WINDOWS_SETUP.md)
