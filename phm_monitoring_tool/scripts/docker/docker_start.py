# docker_init.py

import docker
from scripts.log_manager import log_message

def initialize_docker_client():
    try:
        client = docker.from_env()
        return client
    except Exception as e:
        log_message(f"FAILURE: {e}")

def connect_to_container_volumes(current_container, container_to_monitor, client):
    try:
        container = client.containers.get(container_to_monitor)
        volumes = container.attrs['Mounts']
        for volume in volumes:
            mount_point = volume['Source']
            destination = volume['Destination']
            current_container.exec_run(f'mount --bind {mount_point} {destination}')
            log_message(f"ACTION: Connected volume {mount_point} to {destination} in {current_container.name}")
    except Exception as e:
        log_message(f"ERROR: connecting to volumes of {container_to_monitor}: {str(e)}")

def get_container_networks(container):
    """Helper function to retrieve the networks a container is connected to."""
    try:
        networks = container.attrs['NetworkSettings']['Networks']
        return networks
    except Exception as e:
        log_message(f"Error getting networks for container {container.name}: {str(e)}")
        return None

def disconnect_from_existing_networks(container, client):
    """Disconnect the container from all networks it is currently connected to."""
    networks = get_container_networks(container)
    for network_name in networks.keys():
        log_message(f"Disconnecting {container.name} from {network_name}")
        client.networks.get(network_name).disconnect(container)

def connect_containers_to_same_network(phm_container, ros_container_name, client):
    try:
        ros_container = client.containers.get(ros_container_name)
        
          # Disconnect PHM from any existing networks
        disconnect_from_existing_networks(phm_container, client)

        # Get the networks the ROS app container is connected to
        ros_networks = get_container_networks(ros_container)

        if not ros_networks:
            log_message(f"ERROR: No networks found for {ros_container_name}")
            return
        
        # Check if the ROS app is only connected to the host network
        if 'host' in ros_networks.keys() and len(ros_networks) == 1:
            log_message(f"ROS app is only connected to the host network, creating a new network.")

            # Create a new bridge network
            new_network = client.networks.create("phm_shared_network", driver="bridge")

            # Connect both containers to the new network
            new_network.connect(ros_container)
            new_network.connect(phm_container)

            log_message(f"SUCCESS: Both containers connected to the new network: phm_shared_network")
            return
        else:
            # Find the first network that the ROS app container is connected to (other than host)
            for network_name in ros_networks.keys():
                if network_name != 'host':
                    log_message(f"Connecting PHM tool to ROS app's network: {network_name}")

                    # Connect the PHM tool to this network
                    network = client.networks.get(network_name)
                    try:
                        network.connect(phm_container)
                        log_message(f"PHM tool successfully connected to {network_name}")
                    except docker.errors.APIError as e:
                        log_message(f"ERROR: Could not connect PHM tool to network {network_name}: {str(e)}")
                    return

    except docker.errors.NotFound as e:
        log_message(f"ERROR: Container not found: {str(e)}")
    except Exception as e:
        log_message(f"ERROR: {str(e)}")