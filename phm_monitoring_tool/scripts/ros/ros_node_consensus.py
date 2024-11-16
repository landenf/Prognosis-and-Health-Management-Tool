import docker
import time
import os

client = docker.from_env()
from scripts.log_manager import log_message

def exec_command_in_container(container_name, command):
    container = client.containers.get(container_name)
    result = container.exec_run(command)
    print(result)
    return result.output.decode('utf-8').strip()

def is_node_running(container_name, node_name):
    command = "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode list'"
    node_list = exec_command_in_container(container_name, command).split('\n')
    return node_name in node_list

def get_running_nodes(container_name):
    command = "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode list'"
    node_list = exec_command_in_container(container_name, command).split('\n')
    return [node[1:] if node.startswith('/') else node for node in node_list]

#Main consensus function
def check_nodes(sys_id, container_name):
    required_nodes = [f'agent_{sys_id}/node_1', f'agent_{sys_id}/node_2', f'agent_{sys_id}/node_3']

    running_nodes = get_running_nodes(container_name)
    all_nodes_running = all(node in running_nodes for node in required_nodes)

    if not all_nodes_running:
        not_running_nodes = [node for node in required_nodes if node not in running_nodes]
        log_message(f"FAILURE: (6) Not all required nodes are running. Missing nodes: {', '.join(not_running_nodes)}")
        return

    log_message(f"SUCCESS: (6) All required nodes are running: {', '.join(required_nodes)}")

    return
