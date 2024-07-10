import docker
import pyudev

def get_container_devices(container_name):
    client = docker.from_env()
    try:
        container = client.containers.get(container_name)
        container_info = container.attrs
        devices = container_info['HostConfig']['Devices']
        return devices
    except docker.errors.NotFound:
        print(f"Container {container_name} not found.")
        return None

def check_physical_devices(container_name):
    devices = get_container_devices(container_name)
    if not devices:
        return "No devices found for the container."

    context = pyudev.Context()
    physical_devices = []

    for device in devices:
        host_path = device['PathOnHost']
        device_found = False
        for dev in context.list_devices(subsystem='tty'):
            if dev.device_node == host_path:
                physical_devices.append(host_path)
                device_found = True
                break
        if not device_found:
            for dev in context.list_devices(subsystem='block'):
                if dev.device_node == host_path:
                    physical_devices.append(host_path)
                    device_found = True
                    break

    if physical_devices:
        return f"Physical devices connected to host ports: {', '.join(physical_devices)}"
    else:
        return "No physical devices connected to the host ports."

if __name__ == "__main__":
    container_name = os.getenv('CONTAINER_TO_MONITOR', 'your_default_container_name')
    print(check_physical_devices(container_name))
