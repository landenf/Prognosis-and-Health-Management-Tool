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
            print(f"Connected volume {mount_point} to {destination} in {current_container.name}")
    except Exception as e:
        print(f"Error connecting to volumes of {container_to_monitor}: {str(e)}")

def connect_to_networks(current_container, container_to_monitor, client):
    try:
        container = client.containers.get(container_to_monitor)
        networks = container.attrs['NetworkSettings']['Networks']
        
        for network_name in networks:
            if network_name == "host":
                # Special handling for host network
                network_mode = "host"
                current_container.update(network_mode=network_mode)
                print(f"Connected {current_container.name} to host network")
            else:
                network = client.networks.get(network_name)
                network.connect(current_container)
                print(f"Connected {current_container.name} to network {network_name}")
    except Exception as e:
        print(f"Error connecting to networks of {container_to_monitor}: {str(e)}")
