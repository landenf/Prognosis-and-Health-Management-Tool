import docker

client = docker.from_env()

def get_container_networks(container_name):
    """Retrieve all networks a container is connected to."""
    container = client.containers.get(container_name)
    return container.attrs['NetworkSettings']['Networks']

def connect_to_network(container, network_name):
    """Connect the container to a specific network."""
    network = client.networks.get(network_name)
    network.connect(container)

def main(container_to_monitor):
    # Get current container
    current_container = client.containers.get(socket.gethostname())

    # Get networks of the container to monitor
    networks = get_container_networks(container_to_monitor)

    # Connect the current container to all networks of the container to monitor
    for network_name in networks:
        connect_to_network(current_container, network_name)
        print(f"Connected to {network_name}.")

if __name__ == '__main__':
    import socket
    # Environment variable that specifies which container to monitor
    container_to_monitor = os.getenv('CONTAINER_TO_MONITOR', 'default_container_name')
    main(container_to_monitor)
