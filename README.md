# Integrated Prognostics and Health Monitoring Tool for UAS Swarm Agents

## Overview

This project focuses on developing an Integrated Prognostics and Health Monitoring (PHM) tool tailored for **Unmanned Aircraft Systems (UAS)**, specifically swarm agents. Swarm UAS missions, which rely on distributed control with no centralized system, present unique challenges in communication, health monitoring, and fault detection. The PHM tool in this project aims to ensure mission success by providing real-time diagnostics and performance reports across both hardware and software layers of UAS swarm agents.

### Key Features

- **Real-time Monitoring**: Continuous monitoring of critical hardware and software components aboard each UAS swarm agent.
- **Fault Detection and Reporting**: Detects and reports issues such as communication breakdowns, hardware failures, and software malfunctions to the Ground Station Control (GSC) in real-time.
- **Scalability**: Designed to support distributed, multi-agent UAS swarms, allowing it to scale across fleets of small UAS with minimal computational and communication overhead.
- **Docker-Based Architecture**: The tool leverages Docker containers to encapsulate UAS control processes and ensure reliable, isolated software environments across swarm agents, enhancing portability and consistency.
- **ROS-Integrated**: Built on the Robot Operating System (ROS), allowing seamless interaction with swarm processes and nodes through ROS topics, services, and messaging.

## Project Goals

The **PHM tool** addresses the following challenges faced during UAS swarm missions:

- **Health Monitoring**: Provides a robust system to monitor both hardware (e.g., autopilots, companion computers) and software (e.g., ROS nodes, Docker containers) in real-time.
- **Communication Reliability**: Monitors and detects faults in communication channels between swarm agents and between agents and GSC. Specifically targets low-bandwidth, asynchronous inter-agent communication networks.
- **Scalability and Resource Efficiency**: Focuses on lightweight, resource-efficient diagnostics, making it ideal for small UAS swarms with limited computational resources. Avoids reliance on machine learning models or complex predictive algorithms, focusing instead on direct fault detection and recovery.
- **Automated Recovery**: Implements recovery mechanisms such as Docker container restarts in case of software or hardware failures, minimizing mission downtime.

## System Architecture

The tool is deployed across multiple layers, each responsible for monitoring specific aspects of UAS operations:

1. **Companion Computer Layer**:
   - Executes high-level control, mission planning, and health monitoring.
   - Sends periodic pings to the GSC to verify operational status and communication health.
   - Detects hardware failures (e.g., disconnection of ports or sensors).

2. **Docker Container Layer**:
   - Runs UAS control processes and PHM tool inside Docker containers, enabling consistent environments across all agents.
   - Utilizes health checks (liveness and readiness probes) to ensure software integrity and container functionality.
   - Monitors connectivity between Docker containers and host devices, ensuring communication channels are active.

3. **ROS Layer**:
   - Monitors and manages ROS nodes that control UAS functions. Each ROS node is responsible for publishing and subscribing to topics that facilitate inter-agent communication.
   - The PHM tool verifies the health of these nodes by auditing topic connections and monitoring traffic to detect discrepancies.
   - Ensures consensus between nodes, confirming that all critical UAS functions are operational and in sync.

## Deployment and Testing

The **PHM tool** is tested in a **Software in the Loop (SITL)** environment to simulate real-world UAS swarm operations without the risks associated with physical deployment. SITL allows for controlled introduction of faults, enabling the PHM system to be stress-tested under both single and multiple fault conditions.

After SITL validation, the system is tested in real-world swarm operations, ensuring that it can effectively detect and mitigate hardware and software failures during live missions.

### Performance Metrics:
- **Uptime and Availability**: Monitors the operational availability of swarm agents.
- **Fault Detection Rate**: Measures the tool’s effectiveness in identifying faults and minimizing false positives.
- **Repair Response Time**: Tracks the time taken to automatically recover from faults (e.g., restarting Docker containers).
- **Network Integrity**: Verifies consistent communication between agents and the GSC.

## Future Directions

The PHM tool provides a foundation for reliable UAS swarm operations, ensuring system integrity and mission success. Future improvements may include:
- Enhanced fault detection algorithms utilizing machine learning for predictive maintenance.
- Expanded support for different UAS platforms and swarm configurations.
- Integration with cloud-based control systems for mission planning and fleet management.

## References

This project is based on the research presented in the following paper:  
**Fogle, L., Phillips, G., Bradley, J.**  
"Integrated Prognostics and Health Monitoring Tool for Software Components Aboard UAS Swarm Agents"  
University of Nebraska-Lincoln, 2024 AIAA SciTech Forum.

