#!/bin/bash

# Exit on error
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set up Python path
export PYTHONPATH="/workspace:${PYTHONPATH}"

# Change to workspace directory
cd /workspace

# Run the bridge
exec mcs-bridge "$@"
