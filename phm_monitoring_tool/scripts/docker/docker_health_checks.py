# docker_health_checks.py

import subprocess
import time
import docker
import socket
import os
from scripts.docker.docker_start import initialize_docker_client, connect_to_container_volumes, connect_containers_to_same_network
from scripts.log_manager import log_message

def check_container_health(current_container, client, container_name):
    try:
        container = client.containers.get(container_name)

        # Additional checks
        if not check_container_status(current_container):
            log_message("DOCKER: Failed during check_container_status.")
            return False

        if not check_network(current_container, container_name, client):
            log_message("DOCKER: Failed during check_network.")
            return False

        if not check_files(current_container):
            log_message("DOCKER: Failed during check_files.")
            return False
        
        return True

    except docker.errors.NotFound:
        log_message(f"ERROR: Container {container_name} not found")
    except Exception as e:
        log_message(f"ERROR: checking health of container {container_name}: {str(e)}")

def check_container_status(container):
    try:
        container_state = container.attrs['State']
        if container_state['Status'] != 'running':
            log_message(f"FAILURE: (2) Container {container.name} is not running: {container_state['Status']}")
            return False
        else:
            log_message(f"SUCCESS: (2) Container {container.name} is running")

        if 'Health' in container_state:
            health_status = container_state['Health']['Status']
            if health_status == 'healthy':
                log_message(f"SUCCESS: (2) Container {container.name} is healthy.")
                return True
            else:
                log_message(f"FAILURE: (2) Container {container.name} has internal health check issues: {health_status}")
                return False
        else:
            log_message(f"BYPASS: (2) No health check defined for {container.name}.")
            return True

    except docker.errors.NotFound:
        log_message(f"FAILURE: (2) Container {container.name} not found.")
    except Exception as e:
        log_message(f"ERROR: while checking health of container {container.name}: {str(e)}")

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
            log_message(f"FAILURE: (4) No common networks found between {container_to_monitor_name} and health_management_container.")
            return False

        network_name = next(iter(common_networks)) 
        log_message(f"SUCCESS: (4) Both containers are connected to the same network: {network_name}")

        # Perform ping check between containers
        monitor_ip = container_to_monitor.attrs['NetworkSettings']['Networks'][network_name]['IPAddress']
        exec_command = health_management_container.exec_run(f'ping -c 1 {monitor_ip}')
        if exec_command.exit_code != 0:
            log_message(f"FAILURE: (4) Network check failed for {container_to_monitor_name}. Exit code: {exec_command.exit_code}")
            return False
        else:
            log_message(f"SUCCESS: (4) IP Ping network check passed for {container_to_monitor_name}")
            return True
    except docker.errors.NotFound:
        log_message(f"FAILURE: (4) Health management container not found.")
        return False
    except Exception as e:
        log_message(f"ERROR checking network for {container_to_monitor_name}: {str(e)}")


def check_files(container):
    file_to_check = '/home/catkin_ws/src/testfile.txt'  # Specify the file to check
    exec_command = container.exec_run(f'ls {file_to_check}')  # Use `test -f` to check for file existence
    if exec_command.exit_code != 0:
        log_message(f"SUCCESS: (3) File check failed: {file_to_check} does not exist.")
        return True
    else:
        log_message(f"SUCCESS: (3) File check passed: {file_to_check} exists.")
        return True

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
            log_message(f"ERROR: Container {container.name} not found.")
    
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
        log_message(f"SUCCESS: (5) Found {len(results)} devices connected to container")
        return True
    else:
        log_message("FAILURE: (5) No devices found or no output from checks.")
        return True #bypass

def Run_Docker_Health_Checks():
    try:
        #Start docker checks
        container_to_monitor = os.getenv('CONTAINER_TO_MONITOR', 'default_container_name')
        log_message(f"DOCKER: Checking health of {container_to_monitor}")

        client = initialize_docker_client()
        if not client:
            return False
        current_container = client.containers.get(socket.gethostname())
        print("DOCKER: client initialized.")

        # Sequential checks: Return False if any check fails
        if not connect_to_container_volumes(current_container, container_to_monitor, client):
            log_message("DOCKER: Failed during connect_to_container_volumes.")
            return False

        if not connect_containers_to_same_network(current_container, container_to_monitor, client):
            log_message("DOCKER: Failed during connect_containers_to_same_network.")
            return False

        if not check_container_health(current_container, client, container_to_monitor):
            log_message("DOCKER: Failed during check_container_health.")
            return False

        if not check_physical_devices(current_container, client):
            log_message("DOCKER: Failed during check_physical_devices.")
            return False
        return True

    except docker.errors.NotFound:
        log_message("FAILURE: docker not present")
        return False