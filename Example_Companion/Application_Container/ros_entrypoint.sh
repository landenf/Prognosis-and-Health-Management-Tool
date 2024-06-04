#!/bin/bash
set -e

# Source the ROS setup script to set up the environment
source "/opt/ros/melodic/setup.bash"

# Start roscore in the background
roscore &

# Wait for roscore to start
sleep 5

# Execute the command passed as arguments to this script
exec "$@"
