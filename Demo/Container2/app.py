import docker


def main():
    print("2 Running")
    global client
    try:
        client = docker.from_env()
        print("Docker client initialized.")
        print(client.containers.get("container1"))
        check_container_health("container1")
    except Exception as e:
        print(f"Error: {e}")

def check_container_health(container_name):
    try:
        container = client.containers.get(container_name)
        
        # Check container status
        container_state = container.attrs['State']
        if container_state['Status'] != 'running':
            print(f"Container {container_name} is not running: {container_state['Status']}")

        # Example checks
        check_files(container)

    except docker.errors.NotFound:
        print(f"Container {container_name} not found")
    except Exception as e:
        print(f"Error checking health of container {container_name}: {str(e)}")

def check_files(container):
    files_to_check = ['/src/app.py']
    exec_command = container.exec_run('ls ' + ' '.join(files_to_check))
    if exec_command.exit_code != 0:
        print(f"File check failed for {container.name}: {exec_command.output.decode()}")
    else:
        print(f"All critical files are present in {container.name}")

    tool = client.containers.get("container2")

    exec_command = tool.exec_run(f'ping -c 1 {"container1"}')
    if exec_command.exit_code != 0:
        print(f"Network check failed for {container.name}")
    else:
        print(f"Network check passed for {container.name}")

if __name__ == "__main__":
    main()
