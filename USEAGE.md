# How to use the PHM Tool

## Assumptions
- Must be testing ros enviorments

## Pre-requisits
- You may start your application container in any way. Docker compose is suggested. 
- Ensure your application container either hosts or connects to a bridged network

## Docker Checks

### Check the health of the app_container
-- Done | tested
- add docker health parameters 

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

# TODO
1) Fix broken checks
3) Logging - data processing - summary
2) see if we can deploy this container
3) do more tests with more nodes
4) setup local logging and server


# rename and fix my network
# add more nodes
# clean up the cod eand make it readable
# move ros app into this repo
