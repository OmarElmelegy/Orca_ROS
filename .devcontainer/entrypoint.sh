#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Source workspace if it exists
if [ -f "/workspaces/Orca_ws/devel/setup.bash" ]; then
    source /workspaces/Orca_ws/devel/setup.bash
fi

# Execute the command passed to this entrypoint
exec "$@"
