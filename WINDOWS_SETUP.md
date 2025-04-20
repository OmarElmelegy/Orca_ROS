# Orca ROV Project: Windows Setup Guide

This guide provides detailed instructions for setting up and testing the Orca ROV project on Windows using Docker.

## Prerequisites

1. **Install Docker Desktop for Windows**
   - Download and install from [Docker Desktop](https://www.docker.com/products/docker-desktop)
   - Make sure to enable WSL 2 if prompted
   - Start Docker Desktop and ensure it's running

2. **Install an X Server for GUI Applications**
   - Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
   - Alternative: [Xming](https://sourceforge.net/projects/xming/)

3. **Install Git for Windows** (optional)
   - Download and install from [Git for Windows](https://gitforwindows.org/)

4. **Install Visual Studio Code** (optional, for development)
   - Download and install from [VS Code](https://code.visualstudio.com/)
   - Install the "Remote - Containers" extension

## Setup Instructions

### Option 1: Using Batch Scripts (Recommended for Most Users)

1. **Build the Docker Image**
   ```
   docker-build.bat
   ```
   This will build the Docker image with all required dependencies.

2. **Start the X Server**
   - Launch VcXsrv (XLaunch)
   - Select "Multiple windows"
   - Set "Display number" to 0
   - Select "Start no client"
   - Check "Disable access control" (important!)
   - Click "Next" and then "Finish"

3. **Run the Docker Container**
   ```
   docker-run-gui.bat
   ```
   This will start the Docker container with GUI support.

4. **Inside the Container: Make Scripts Executable**
   ```bash
   # First make the shell scripts executable
   chmod +x *.sh

   # Then run the script to make all scripts executable
   ./make_scripts_executable.sh
   ```
   This will make all Python scripts and shell scripts executable.

5. **Inside the Container: Make Configuration Files Executable**
   ```bash
   ./make_cfg_executable.sh
   ```
   This will make all configuration files executable.

6. **Inside the Container: Build the Workspace**
   ```
   ./build_workspace.sh
   ```
   This will build the ROS workspace.

### Option 2: Using VS Code (Recommended for Developers)

1. **Open the Project in VS Code**
   - Open VS Code
   - Click "File" > "Open Folder" and select the "Orca_ws" folder

2. **Reopen in Container**
   - VS Code should detect the .devcontainer folder and prompt you to "Reopen in Container"
   - Alternatively, press F1 and select "Remote-Containers: Reopen in Container"
   - VS Code will build the Docker image and start the container

3. **Inside VS Code Terminal: Make Scripts Executable**
   ```bash
   # First make the shell scripts executable
   chmod +x *.sh

   # Then run the script to make all scripts executable
   ./make_scripts_executable.sh
   ```

4. **Inside VS Code Terminal: Make Configuration Files Executable**
   ```bash
   ./make_cfg_executable.sh
   ```

5. **Inside VS Code Terminal: Build the Workspace**
   ```bash
   ./build_workspace.sh
   ```

## Testing the System

### Setting Up X Server for GUI Applications

Before running any GUI applications (like RViz or Gazebo), make sure:

1. VcXsrv is running with "Disable access control" checked
2. You're using the `docker-run-gui.bat` script to start the container

### Testing Without RViz (Recommended for Initial Test)

If you're having issues with RViz, you can test the system without it:

```bash
# Using the test script
./test-model-only.sh

# Or manually
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch orca_description display.launch rviz:=false
```

This will load the ROV model and start the joint_state_publisher_gui without RViz.

### Testing with Simple RViz Configuration

If you want to test RViz with a simple configuration:

```bash
# Using the test script with default RViz configuration
./test-rviz-simple.sh

# Using the test script with custom simple configuration
./test-rviz-custom.sh

# Using the test script with minimal configuration
./test-rviz-minimal.sh

# Using the test script with software rendering explicitly enabled
./test-rviz-software.sh

# Or manually
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun rviz rviz -d /opt/ros/noetic/share/rviz/default.rviz
```

### Testing RViz with the ROV Model

Once you've confirmed that RViz works with a simple configuration, you can try loading the ROV model:

```bash
# Using the test script
./test-rviz.sh

# Or manually
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch orca_description display.launch
```

This will open RViz with the Orca ROV model loaded.

### Testing Gazebo Simulation

Inside the Docker container:

```bash
# Using the test script
./test-gazebo.bat

# Or manually
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch orca_bringup orca_simulation.launch
```

This will open Gazebo with the Orca ROV model in an underwater environment.

### Testing Missions

Inside the Docker container:

```bash
# Test water sampling mission
./test-mission.sh water

# Test object retrieval mission
./test-mission.sh object

# Or manually
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch orca_bringup water_sampling_mission.launch
# or
roslaunch orca_bringup object_retrieval_mission.launch
```

## Troubleshooting

### Docker Build Issues

1. **Package not found errors**:
   - The Dockerfile has been updated to use the correct ROS package names
   - `python3-smach` and `python3-smach-ros` packages have been replaced with `ros-noetic-executive-smach`, `ros-noetic-smach-ros`, and `ros-noetic-smach-viewer`
   - `ros-noetic-v4l2-camera` package has been removed as it's not available in the standard ROS Noetic repositories
   - Added `libeigen3-dev` package for the Eigen library used by the thruster allocator

2. **Network issues during build**:
   - Make sure you have a stable internet connection
   - Try building again later if Docker Hub or other repositories are temporarily unavailable

3. **Disk space issues**:
   - Docker images can be large. Make sure you have at least 10GB of free disk space
   - You can clean up unused Docker images with `docker system prune -a`

### Build Issues

1. **Missing configuration files**:
   - Make sure to run `./make_cfg_executable.bat` before building the workspace
   - This will make all configuration files executable

2. **Missing MS5837 implementation**:
   - The CMakeLists.txt has been updated to compile the MS5837.cpp file and link it to the pressure_sensor_node
   - Fixed a naming conflict with the system `read()` function by using the global scope operator (`::`)

3. **Missing Eigen library**:
   - The Dockerfile has been updated to include the libeigen3-dev package
   - The CMakeLists.txt has been updated to find and include the Eigen library

4. **Missing dependencies**:
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
   - Use the `test-rviz-custom.sh` script to test RViz with a custom simple configuration
   - Use the `test-rviz-minimal.sh` script to test RViz with a minimal configuration
   - Use the `test-rviz-software.sh` script to test RViz with software rendering explicitly enabled

3. **Incremental Testing**:
   - First test without RViz
   - Then test RViz with a simple configuration
   - Finally test RViz with the ROV model

### X Server Issues

1. **Check if VcXsrv is running**
   - Look for the VcXsrv icon in the system tray
   - Make sure "Disable access control" was checked when starting

2. **Test with a simple X application**
   ```bash
   xeyes
   ```

3. **Check DISPLAY environment variable**
   ```bash
   echo $DISPLAY
   ```
   It should be set to `host.docker.internal:0.0`

4. **Try restarting with explicit display settings**
   ```bash
   export DISPLAY=host.docker.internal:0.0
   ```

### Docker Issues

1. **Docker not running**
   - Make sure Docker Desktop is running
   - Check Docker status in system tray

2. **Permission issues**
   - Run Command Prompt or PowerShell as Administrator

3. **Container not starting**
   - Check Docker Desktop logs
   - Try restarting Docker Desktop

### ROS Issues

1. **ROS nodes not communicating**
   ```bash
   # Check ROS master
   rostopic list

   # Set ROS_MASTER_URI if needed
   export ROS_MASTER_URI=http://localhost:11311
   ```

2. **Missing dependencies**
   ```bash
   # Inside the container
   apt-get update
   apt-get install ros-noetic-PACKAGE-NAME
   ```

## Advanced: Customizing the Docker Environment

If you need to customize the Docker environment:

1. Edit the `.devcontainer/Dockerfile` to add more dependencies
2. Rebuild the Docker image using `docker-build.bat`
3. Run the container using `docker-run-gui.bat`

## Additional Resources

- [ROS Wiki](http://wiki.ros.org/)
- [Docker Documentation](https://docs.docker.com/)
- [VS Code Remote Containers](https://code.visualstudio.com/docs/remote/containers)
- [VcXsrv Documentation](https://sourceforge.net/projects/vcxsrv/)
