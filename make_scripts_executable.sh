#!/bin/bash
# This script should be run inside the Docker container
echo "Making scripts executable..."

# Find all Python scripts and make them executable
find . -name "*.py" -exec chmod +x {} \;

# Make all shell scripts executable
chmod +x *.sh

echo "All scripts are now executable."
