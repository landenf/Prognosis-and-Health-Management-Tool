# How to use the PHM Tool

## Assumptions
- Must be testing ros enviorments

## Pre-requisits
- You may start your application container in any way. Docker compose is suggested. 
- Ensure your application container either hosts or connects to a bridged network

## Startup Simulation

`cd into root directory` 

`sudo docker compose up` 

This will start a simulation as outlined in the docker-compose.yaml which runs two ros_apps's and two phm containers which simulates two agents running. 