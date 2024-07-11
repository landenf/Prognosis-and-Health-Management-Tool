# How to use the PHM Tool

## Assumptions
- Must be testing ros enviorments

## Pre-requisits
- You may start your application container in any way. Docker compose is suggested. 
- Ensure your application container either hosts or connects to a bridged network

## Docker Checks

### Check the health of the app_container
-- Done and tested

### Check that the app_container is connected to the correct volumes
-- Done not tested

### Check that the app_container is connected to the same network
-- Done not tested

### Check the app_container can see attached physical devices
- Work in progress

## ROS Checks

### Check that all nodes are in consensus that the correct nodes are running
-- Work in progress

### Check that all req. topics are connected
-- Work in progress

### Check that topics are recieivng information and alert when not. 
-- Work in progress

# TODO
1) Fix broken checks
2) see if we can deploy this container
3) do more tests with more nodes
4) setup local logging and server