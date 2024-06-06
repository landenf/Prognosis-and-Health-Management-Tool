import docker
import os
import sys
import time



# Container to monitor
container_to_monitor = os.getenv('CONTAINER_TO_MONITOR', 'containera')

def check_container_health(container_name):
    try:
        container = client.containers.get(container_name)
        
        # Check container status
        container_state = container.attrs['State']
        if container_state['Status'] != 'running':
            print(f"Container {container_name} is not running: {container_state['Status']}")
            sys.exit(1)

        # Example checks
        check_files(container)
        check_network(container)
        #check_ports(container)

    except docker.errors.NotFound:
        print(f"Container {container_name} not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error checking health of container {container_name}: {str(e)}")
        sys.exit(1)

def check_files(container):
    # Implement checks for files and directories inside the container
    files_to_check = ['/app/app.py', '/app/requirements.txt']
    exec_command = container.exec_run('ls ' + ' '.join(files_to_check))
    if exec_command.exit_code != 0:
        print(f"File check failed for {container.name}: {exec_command.output.decode()}")
        sys.exit(1)
    else:
        print(f"All critical files are present in {container.name}")

def check_network(container):
    # Implement network connectivity checks here
    #container = client.containers.get(container_Name)
    exec_command = container.exec_run(f'ping -c 1 {container_to_monitor}')
    if exec_command.exit_code != 0:
        print(f"Network check failed for {container.name}")
        sys.exit(1)
    else:
        print(f"Network check passed for {container.name}")

def check_ports(container):
    # Implement port device checks here
    ports_to_check = [80]
    for port in ports_to_check:
        exec_command = container.exec_run(f'netstat -an | grep :{port}')
        if exec_command.exit_code != 0:
            print(f"Port {port} check failed for {container.name}")
            sys.exit(1)
        else:
            print(f"Port {port} check passed for {container.name}")

if __name__ == "__main__":
    while True:
        print("Running B")
        # Docker client
        try:
            client = docker.from_env()
            print("Docker client initialized.")
        except Exception as e:
            print(f"Error: {e}")
        check_container_health(container_to_monitor)
        time.sleep(30)  # Interval between health checks
