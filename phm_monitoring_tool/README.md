# Health Measure Overview

## High Level of Each Health Check

1) **Companion Computer**  
   This check ensures that the base companion computer is alive and tracking back a heartbeat. 

2) **Monioriting Container ss Healthy**
   This check ensure the health check the user created on their container is marked as healthy. 

3) **Present and Executable Files**  
   This check verifies that all required ROS scripts and executable files are present and have the correct permissions.

4) **Network Check**  
   This check ensures that the **PHM tool** container and the **ROS app** container are on the same network and can communicate with each other.

5) **Critical External Hardware Connected**  
   This check verifies that any critical external hardware (e.g., sensors or USB devices) required for operation is connected and recognized by the system.

6) **ROS Node Consensus**  
   This check ensures that all critical ROS nodes are active and agree on the system state, confirming consistent behavior across nodes.

7) **Critical Node Topic Connections**  
   This check verifies that critical ROS nodes are properly subscribed to and publishing on the required topics.

8) **Topic Traffic Monitoring**  
   This check monitors the traffic on key ROS topics to ensure that data is being transmitted and received at the expected rates.


## How Each Health Check Works

4) **Network Check**  
   - **Initial Check**: The system first inspects the network that the **ROS app** container is connected to. If the **ROS app** is connected to any network (other than the `host` network), the **PHM tool** container is dynamically connected to the same network.
   - **Handling Host Network**: If the **ROS app** container is only on the `host` network, a new bridge network (`phm_shared_network`) is created, and both the **PHM tool** and the **ROS app** containers are connected to this new network. This ensures that they can communicate over a proper Docker bridge network rather than relying on the `host` network, which does not allow for container-to-container networking.
   - **Network Disconnection**: Before connecting the **PHM tool** to the appropriate network, it is first disconnected from any existing networks to avoid potential conflicts and ensure it's only connected to the intended network.
   - **Ping Test**: Once both containers are on the same network, the **PHM tool** performs a ping test to the **ROS app** container’s IP address to verify that the network communication is functioning as expected.

