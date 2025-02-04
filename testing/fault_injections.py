import docker
import sys
import time
from datetime import datetime

# Hardcoded variables
CONTAINER_NAME = "ros_app_container_1"
FILE_PATH_TO_REMOVE = "/home/catkin_ws/src/testfile.txt"

# Log file path
log_file_path = "./container_actions.log"

def log_action(action_description):
    """Log the action with a timestamp to a file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {action_description}\n"
    with open(log_file_path, "a") as log_file:
        log_file.write(log_entry)
    print(log_entry.strip())  # Also print to console

def remove_container_from_network(client):
    try:
        container = client.containers.get(CONTAINER_NAME)
        networks = container.attrs['NetworkSettings']['Networks']
        for network in networks.keys():
            action_description = f"Removing container '{CONTAINER_NAME}' from network '{network}'..."
            log_action(action_description)
            client.networks.get(network).disconnect(CONTAINER_NAME)
        log_action(f"Container '{CONTAINER_NAME}' removed from all networks.")
    except Exception as e:
        log_action(f"Error removing container from network: {e}")
        sys.exit(1)

def remove_and_restore_file(client):
    try:
        container = client.containers.get(CONTAINER_NAME)
        
        # Step 1: Remove the file
        exit_code, output = container.exec_run(f"rm -f {FILE_PATH_TO_REMOVE}")
        if exit_code == 0:
            log_action(f"File '{FILE_PATH_TO_REMOVE}' removed from container '{CONTAINER_NAME}'.")
        else:
            log_action(f"Failed to remove file '{FILE_PATH_TO_REMOVE}': {output.decode('utf-8')}")
            sys.exit(1)
        
        # Step 2: Wait for one minute
        log_action("Waiting for 1 minute...")
        time.sleep(60)
        
        # Step 3: Recreate the file
        exit_code, output = container.exec_run(f"touch {FILE_PATH_TO_REMOVE}")
        if exit_code == 0:
            log_action(f"File '{FILE_PATH_TO_REMOVE}' restored in container '{CONTAINER_NAME}'.")
        else:
            log_action(f"Failed to restore file '{FILE_PATH_TO_REMOVE}': {output.decode('utf-8')}")
            sys.exit(1)
        
    except Exception as e:
        log_action(f"Error during file removal/restoration: {e}")
        sys.exit(1)

def mark_container_as_unhealthy(client):
    try:
        container = client.containers.get(CONTAINER_NAME)
        # Update the container's healthcheck to always fail
        container.update(healthcheck={
            "Test": ["CMD-SHELL", "exit 1"],  # CMD-SHELL "exit 1" always fails
            "Interval": 1000000000,  # Interval in nanoseconds (1 second here)
            "Timeout": 1000000000,   # Timeout for the check (1 second here)
            "Retries": 3             # Number of retries before marking unhealthy
        })
        log_action(f"Container '{CONTAINER_NAME}' marked as unhealthy.")
    except Exception as e:
        log_action(f"Error marking container as unhealthy: {e}")
        sys.exit(1)

def kill_ros_app_container(client):
    try:
        container = client.containers.get(CONTAINER_NAME)
        container.kill()
        log_action(f"Container '{CONTAINER_NAME}' has been killed.")
    except Exception as e:
        log_action(f"Error killing container '{CONTAINER_NAME}': {e}")
        sys.exit(1)

def kill_ros_node(client):
    ROS_COMMAND = "rosnode kill /agent_1/node_1"

    try:        
        # Get the container
        container = client.containers.get(CONTAINER_NAME)
        
        # # Execute the command inside the container
        exit_code, output = container.exec_run("/bin/bash -c 'source /opt/ros/noetic/setup.bash && " + ROS_COMMAND + "'")
        
        # Check the result
        if exit_code == 0:
            log_action(f"Command executed successfully: {output.decode('utf-8')}")
        else:
            log_action(f"Command failed with exit code {exit_code}: {output.decode('utf-8')}")
    except Exception as e:
        log_action(f"Error executing command in container: {e}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        log_action("Invalid arguments. Usage: python manage_container.py <check_number>")
        print("1 - Remove container from network")
        print("2 - Remove and restore file inside container")
        print("3 - Mark container as unhealthy")
        print("4 - Kill the ROS app container")
        print("5 - Kill a ROS node")
        sys.exit(1)

    check_number = sys.argv[1]

    # Initialize Docker client
    client = docker.DockerClient(base_url="unix://var/run/docker.sock")

    if check_number == "1":
        remove_container_from_network(client)
    elif check_number == "2":
        remove_and_restore_file(client)
    elif check_number == "3":
        mark_container_as_unhealthy(client)
    elif check_number == "4":
        kill_ros_app_container(client)
    elif check_number == "5":
        kill_ros_node(client)
    else:
        log_action("Invalid check number. Please use 1, 2, 3, or 4.")
        sys.exit(1)
