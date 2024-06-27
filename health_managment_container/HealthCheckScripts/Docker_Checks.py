import docker

def initialize_docker_client():
    try:
        client = docker.from_env()
        return client
    except Exception as e:
        print(f"Error: {e}")

def check_container_health(client, container_name):
    try:
        container = client.containers.get(container_name)

        check_container_status(container)
        check_network(container, client)
        check_files(container)
        check_ports(container)

    except docker.errors.NotFound:
        print(f"Container {container_name} not found")
    except Exception as e:
        print(f"Error checking health of container {container_name}: {str(e)}")

def check_container_status(container):
    try:
        #Ensure container is running
        container_state = container.attrs['State']
        if container_state['Status'] != 'running':
            print(f"Container {container.name} is not running: {container_state['Status']}")
            return  
        else:
            print(f"Container {container.name} is running")

        #Check internal docker health measures and pre-defined health checks
        if 'Health' in container_state:
            health_status = container_state['Health']['Status']
            if health_status == 'healthy':
                print(f"Container {container.name} is healthy.")
            else:
                print(f"Container {container.name} has health issues: {health_status}")
        else:
            print(f"No health check defined for {container.name}.")

    except docker.errors.NotFound:
        print(f"Container {container.name} not found.")
    except Exception as e:
        print(f"Error checking health of container {container.name}: {str(e)}")

def check_network(container, client):
    #ensure connectivity between PHM tool and application container
    this = client.containers.get("health_managment_container")
    exec_command = this.exec_run(f'ping -c 1 {container.name}')
    if exec_command.exit_code != 0:
        print(f"Network check failed for {container.name}")
    else:
        print(f"Network check passed for {container.name}")

def check_files(container):
    #ensure expected important files are present
    files_to_check = ['/src/app.py']
    exec_command = container.exec_run('ls ' + ' '.join(files_to_check))
    if exec_command.exit_code != 0:
        print(f"File check failed for {container.name}: {exec_command.output.decode()}")
    else:
        print(f"All critical files are present in {container.name}")

def check_ports(container):
    #ensure connected hardware
    ports_to_check = [80]
    for port in ports_to_check:
        exec_command = container.exec_run(f'netstat -an | grep :{port}')
        if exec_command.exit_code != 0:
            print(f"Port {port} check failed for {container.name}")
        else:
            print(f"Port {port} check passed for {container.name}")

def Run_Docker_Health_Checks():
    client = initialize_docker_client()
    print("Docker client initialized.")
    
    containers_to_check = ["ros_app_container"]
    for container_name in containers_to_check:
        print(f"Checking health of {container_name}")
        check_container_health(client, container_name)

if __name__ == "__main__":
    Run_Docker_Health_Checks()
