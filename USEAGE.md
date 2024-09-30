# How to use the PHM Tool

## Assumptions
- Must be testing ros enviorments

## Pre-requisits
- You may start your application container in any way. Docker compose is suggested. 
- Ensure your application container either hosts or connects to a bridged network

## Docker Checks

### Check the health of the app_container
-- Done 

### Check that the app_container is connected to the correct volumes
-- Done 

### Check that the app_container is connected to the same network
-- WORK IN PROGRESS

### Check the app_container can see attached physical devices
- Done

## ROS Checks

### Check that all nodes are in consensus that the correct nodes are running
-- NODES SEE EACHOTHER HELP

### Check that all req. topics are connected
-- Done

### Check that topics are recieivng information and alert when not. 
-- Done

