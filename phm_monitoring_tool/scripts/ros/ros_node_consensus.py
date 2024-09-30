import docker
import time
import os

client = docker.from_env()

def exec_command_in_container(container_name, command):
    container = client.containers.get(container_name)
    result = container.exec_run(command)
    return result.output.decode('utf-8').strip()

def is_node_running(container_name, node_name):
    command = "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode list'"
    node_list = exec_command_in_container(container_name, command).split('\n')
    return node_name in node_list

def get_running_nodes(container_name):
    command = "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode list'"
    node_list = exec_command_in_container(container_name, command).split('\n')
    return node_list

#Main consensus function
def check_consensus():
    required_nodes = ['/node_1', '/node_2', '/node_3']
    container_name = os.getenv('CONTAINER_TO_MONITOR')

    running_nodes = get_running_nodes(container_name)
    print(running_nodes)
    all_nodes_running = all(node in running_nodes for node in required_nodes)

    if not all_nodes_running:
        not_running_nodes = [node for node in required_nodes if node not in running_nodes]
        print(f"FAILURE: Not all required nodes are running. Missing nodes: {', '.join(not_running_nodes)}")
        return

    print(f"SUCCESS: All required nodes are running: {', '.join(required_nodes)}")

    all_nodes_can_see_each_other = True
    
    for node in required_nodes:
        # Run `rosnode info` for the current node
        command = f"rosnode info {node}"
        node_info = exec_command_in_container(container_name, command)
        print(node_info)
        # Print the information for debugging
        print(f"Info for {node}:\n{node_info}")
        
        # Check if all other nodes are mentioned in the info
        for other_node in required_nodes:
            if other_node != node and other_node not in node_info:
                print(f"{node} cannot see {other_node}.")
                all_nodes_can_see_each_other = False

    if all_nodes_can_see_each_other:
        print("SUCCESS: All nodes can see each other.")
    else:
        print("FAILURE: Not all nodes can see each other.")
    return
