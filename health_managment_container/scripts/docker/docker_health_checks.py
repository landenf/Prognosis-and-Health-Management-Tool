# docker_health_checks.py

import subprocess
import docker
import socket
import os
from scripts.docker.docker_start import initialize_docker_client, connect_to_container_volumes, connect_to_networks

def check_container_health(client, container_name):
    try:
        container = client.containers.get(container_name)

        check_container_status(container)
        check_network(container, client)
        check_files(container)

    except docker.errors.NotFound:
        print(f"ERROR: Container {container_name} not found")
    except Exception as e:
        print(f"ERROR: checking health of container {container_name}: {str(e)}")

def check_container_status(container):
    try:
        container_state = container.attrs['State']
        if container_state['Status'] != 'running':
            print(f"WARNING: Container {container.name} is not running: {container_state['Status']}")
            return
        else:
            print(f"SUCCESS: Container {container.name} is running")

        if 'Health' in container_state:
            health_status = container_state['Health']['Status']
            if health_status == 'healthy':
                print(f"SUCCESS: Container {container.name} is healthy.")
            else:
                print(f"WARNING: Container {container.name} has health issues: {health_status}")
        else:
            print(f"BYPASS: No health check defined for {container.name}.")

    except docker.errors.NotFound:
        print(f"WARNING: Container {container.name} not found.")
    except Exception as e:
        print(f"ERROR: while checking health of container {container.name}: {str(e)}")

def check_network(container, client):
    try:
        current_container_id = socket.gethostname()  # Get the current container ID or hostname
        health_management_container = client.containers.get(current_container_id)
        networks = container.attrs['NetworkSettings']['Networks']
        health_management_networks = health_management_container.attrs['NetworkSettings']['Networks']
        
        common_networks = set(networks.keys()).intersection(health_management_networks.keys())
        
        if not common_networks:
            print(f"FAILURE: No common networks found between {container.name} and health_management_container.")
            return
        
        exec_command = health_management_container.exec_run(f'ping -c 1 {container.name}')
        if exec_command.exit_code != 0:
            print(f"FAILURE: Network check failed for {container.name}. Exit code: {exec_command.exit_code}")
            print(f"Output: {exec_command.output.decode()}")
        else:
            print(f"SUCCESS: Network check passed for {container.name}")
    except docker.errors.NotFound:
        print(f"WARNING: Health management container not found.")
    except Exception as e:
        print(f"ERROR checking network for {container.name}: {str(e)}")

def check_files(container):
    try:
        path_to_check = '/home/catkin_ws/src/python_package_example'
        exec_command = container.exec_run(f'ls {path_to_check}')
        if exec_command.exit_code != 0:
            print(f"FAILED: File check failed: {path_to_check} does not exist.")
        else:
            print(f"SUCCESS: File check passed: {path_to_check} exists.")
    except Exception as e:
        print(f"Error checking files: {str(e)}")

def check_physical_devices(container, client):

    def get_container_devices(container):
        try:
            container_info = container.attrs
            devices = container_info['HostConfig']['Devices']
            return devices
        except docker.errors.NotFound:
            print(f"Container {container.name} not found.")
            return None
    
    devices = get_container_devices(container)
    if not devices:
        print("No devices found for the container.")
        return

    physical_devices = []

    # Get list of block devices using lsblk // todo not sure how to do this
    lsblk_output = subprocess.check_output(['lsblk', '-dn', '-o', 'NAME']).decode().split()

    for device in devices:
        host_path = device['PathOnHost']
        device_name = host_path.split('/')[-1]
        if device_name in lsblk_output:
            physical_devices.append(host_path)

    if physical_devices:
        return f"Physical devices connected to host ports: {', '.join(physical_devices)}"
    else:
        return "No physical devices connected to the host ports."

def Run_Docker_Health_Checks():
    #Start docker checks
    container_to_monitor = os.getenv('CONTAINER_TO_MONITOR', 'default_container_name')
    print(f"DOCKER: Checking health of {container_to_monitor}")

    client = initialize_docker_client()
    if not client:
        return
    current_container = client.containers.get(socket.gethostname())
    print("Docker client initialized.")

    connect_to_container_volumes(current_container, container_to_monitor, client)
    connect_to_networks(current_container, container_to_monitor, client)
    check_container_health(client, container_to_monitor)
    check_physical_devices(current_container, client)