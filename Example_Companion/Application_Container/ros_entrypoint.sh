#!/bin/bash
# Start ROS core
roscore &
sleep 5
# Run the application
exec "$@"
