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

        # Check if PHM tool is on 'my_network', and ensure it is.
        if 'my_network' not in health_management_networks:
            print(f"ACTION: PHM container not on 'my_network'. Connecting PHM container to 'my_network'.")
            client.networks.get('my_network').connect(health_management_container)

        # Check if the monitored container is on 'my_network', and ensure it is.
        if 'my_network' not in monitor_networks:
            print(f"ACTION: Monitored container not on 'my_network'. Connecting monitored container to 'my_network'.")
            client.networks.get('my_network').connect(container_to_monitor)
        else:
            print(f"SUCCESS: Monitored container is already on 'my_network'.")

    except docker.errors.NotFound as e:
        print(f"ERROR: Container not found. {str(e)}")
    except Exception as e:
        print(f"ERROR: {str(e)}")