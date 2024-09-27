# docker_init.py

import docker
import os
import socket

def initialize_docker_client():
    try:
        client = docker.from_env()
        return client
    except Exception as e:
        print(f"Error: {e}")

def connect_to_container_volumes(current_container, container_to_monitor, client):
    try:
        container = client.containers.get(container_to_monitor)
        volumes = container.attrs['Mounts']
        for volume in volumes:
            mount_point = volume['Source']
            destination = volume['Destination']
            current_container.exec_run(f'mount --bind {mount_point} {destination}')
            print(f"ACTION: Connected volume {mount_point} to {destination} in {current_container.name}")
    except Exception as e:
        print(f"ERROR: connecting to volumes of {container_to_monitor}: {str(e)}")

def connect_containers_to_same_network(health_management_container, container_to_monitor_name, client):
    try:
        container_to_monitor = client.containers.get(container_to_monitor_name)

        # Get network settings for both containers
        monitor_networks = container_to_monitor.attrs['NetworkSettings']['Networks']
        health_management_networks = health_management_container.attrs['NetworkSettings']['Networks']

        # Case where one container is on the host network
        if 'host' in monitor_networks and 'host' not in health_management_networks:
            print(f"Connecting PHM container to host network.")
            client.networks.get('host').connect(health_management_container)
            return
        elif 'host' in health_management_networks and 'host' not in monitor_networks:
            print(f"Connecting monitored container to host network.")
            client.networks.get('host').connect(container_to_monitor)
            return

        # If neither container is on the host network, connect both to the host network
        if 'host' not in monitor_networks and 'host' not in health_management_networks:
            print(f"Connecting both containers to the host network.")
            client.networks.get('host').connect(health_management_container)
            client.networks.get('host').connect(container_to_monitor)
            return

        # Ensure both are on the same external network if host network isn't used
        common_networks = set(monitor_networks.keys()).intersection(health_management_networks.keys())
        if not common_networks:
            # Connect both containers to the same external network if no common networks exist
            for network_name in monitor_networks:
                print(f"Connecting PHM container to {network_name}.")
                client.networks.get(network_name).connect(health_management_container)
            return
        else:
            print(f"SUCCESS: Both containers are already on the same network(s): {', '.join(common_networks)}")

    except docker.errors.NotFound as e:
        print(f"ERROR: Container not found. {str(e)}")
    except Exception as e:
        print(f"ERROR: {str(e)}")