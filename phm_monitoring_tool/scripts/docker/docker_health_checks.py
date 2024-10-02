# docker_health_checks.py

import subprocess
import time
import docker
import socket
import os
from scripts.docker.docker_start import initialize_docker_client, connect_to_container_volumes, connect_containers_to_same_network

def check_container_health(current_container, client, container_name):
    try:
        container = client.containers.get(container_name)

        check_container_status(container)
        check_network(current_container, container_name, client)
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

def check_network(health_management_container, container_to_monitor_name, client):
    try:
        health_management_container = client.containers.get(health_management_container.name) # refresh getting container
        container_to_monitor = client.containers.get(container_to_monitor_name)

        # Get the network settings for both containers
        monitor_networks = container_to_monitor.attrs['NetworkSettings']['Networks']
        health_management_networks = health_management_container.attrs['NetworkSettings']['Networks']


        # Check for common networks dynamically (avoid hardcoding network names)
        common_networks = set(monitor_networks.keys()).intersection(health_management_networks.keys())

        if not common_networks:
            print(f"FAILURE: No common networks found between {container_to_monitor_name} and health_management_container.")
            return

        network_name = next(iter(common_networks)) 
        print(f"SUCCESS: Both containers are connected to the same network: {network_name}")

        # Perform ping check between containers
        monitor_ip = container_to_monitor.attrs['NetworkSettings']['Networks'][network_name]['IPAddress']
        exec_command = health_management_container.exec_run(f'ping -c 1 {monitor_ip}')
        if exec_command.exit_code != 0:
            print(f"FAILURE: Network check failed for {container_to_monitor_name}. Exit code: {exec_command.exit_code}")
            print(f"Output: {exec_command.output.decode()}")
        else:
            print(f"SUCCESS: IP Ping network check passed for {container_to_monitor_name}")
    except docker.errors.NotFound:
        print(f"WARNING: Health management container not found.")
    except Exception as e:
        print(f"ERROR checking network for {container_to_monitor_name}: {str(e)}")


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
    results = []
    
    def get_container_devices(container):
        try:
            container_info = container.attrs
            devices = container_info['HostConfig']['Devices']
            if devices:
                for device in devices:
                    results.append(f"Container Device: {device['PathOnHost']} mapped to {device['PathInContainer']}")
            else:
                results.append("WARNING: No devices explicitly passed to the container.")
        except docker.errors.NotFound:
            print(f"ERROR: Container {container.name} not found.")
    
    def run_command(command):
        try:
            output = subprocess.check_output(command, shell=True).decode().splitlines()
            return output if output else []
        except subprocess.CalledProcessError:
            return []
    
    get_container_devices(container)
    
    lsusb_output = run_command("lsusb")
    if lsusb_output:
        results.append("USB Devices (lsusb):")
        results.extend(lsusb_output)
    
    lsblk_output = run_command("lsblk -o NAME,SIZE,TYPE,MOUNTPOINT")
    if lsblk_output:
        results.append("Block Devices (lsblk):")
        results.extend(lsblk_output)
    
    lspci_output = run_command("lspci")
    if lspci_output:
        results.append("PCI Devices (lspci):")
        results.extend(lspci_output)

    if results:
        print("\n".join(results))
    else:
        print("No devices found or no output from checks.")

def Run_Docker_Health_Checks():
    #Start docker checks
    container_to_monitor = os.getenv('CONTAINER_TO_MONITOR', 'default_container_name')
    print(f"DOCKER: Checking health of {container_to_monitor}")

    client = initialize_docker_client()
    if not client:
        return
    current_container = client.containers.get(socket.gethostname())
    print("DOCKER: client initialized.")

    connect_to_container_volumes(current_container, container_to_monitor, client)
    connect_containers_to_same_network(current_container, container_to_monitor, client)
    check_container_health(current_container, client, container_to_monitor)
    check_physical_devices(current_container, client)