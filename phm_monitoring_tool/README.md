# Health Measure Overview

## High Level of Each Health Check

3) **Network Check**  
   This check ensures that the **PHM tool** container and the **ROS app** container are on the same network and can communicate with each other. 

## How Each Health Check Works

3) **Network Check**  
   - **Initial Check**: The system first inspects the network that the **ROS app** container is connected to. If the **ROS app** is connected to any network (other than the `host` network), the **PHM tool** container is dynamically connected to the same network.
   - **Handling Host Network**: If the **ROS app** container is only on the `host` network, a new bridge network (`phm_shared_network`) is created, and both the **PHM tool** and the **ROS app** containers are connected to this new network. This ensures that they can communicate over a proper Docker bridge network rather than relying on the `host` network, which does not allow for container-to-container networking.
   - **Network Disconnection**: Before connecting the **PHM tool** to the appropriate network, it is first disconnected from any existing networks to avoid potential conflicts and ensure it's only connected to the intended network.
   - **Ping Test**: Once both containers are on the same network, the **PHM tool** performs a ping test to the **ROS app** container’s IP address to verify that the network communication is functioning as expected.

