## /communication 
- In some cases communication can not be facilitated through the MAVLINK infrustructure while using SITL instances. This container is meant to emulate a mavlink communication but within the ROS nodes itself. This container is spun up from the docker file. 

## /ground_station_control
-  Ground station control represents an external ground station script listening for updates published from the swarm containing the swarm consensus. This script can just be run in the background and listen for updates.

## /logs
- The logs folder contains simulated agents files containing logs that would be saved on a live agent. Thease files would persist outside of the network and simualtion to preserve detailed logs for inspection post flight.

## /phm_monitoring_tool
- This container it the research based container containg all of the logic and scripting for the swarm health monitoring. In the scripts folder, there are scripts for both ROS and Docker seperatly. The main.py script consists of a loop that runs all health checks periodically. 

## /ros_app
- This container acts as a standalone ros application simulating a drone. Two packages are included for either multiple nodes or a single node. Thease are then manipulated with testing to throw different use cases at the health monitor. One ros app needs to be spun up for each agent being simulated. 

## /testing
- Testing includes scripts for inserting faults into agents and monioring the responses to those faults. 

### Research.md

### Structure.md

### Usage.md