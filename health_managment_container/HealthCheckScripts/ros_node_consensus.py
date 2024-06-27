import docker
import rospy
from std_msgs.msg import String

# Initialize Docker client
client = docker.from_env()

# Function to execute a command in a specific container
def exec_command_in_container(container_name, command):
    container = client.containers.get(container_name)
    result = container.exec_run(command)
    return result.output.decode('utf-8').strip()

# Function to check if a ROS node is running
def is_node_running(container_name, node_name):
    command = f"rosnode list"
    node_list = exec_command_in_container(container_name, command).split('\n')
    return node_name in node_list

# Function to get running nodes
def get_running_nodes(container_name):
    command = "rosnode list"
    node_list = exec_command_in_container(container_name, command).split('\n')
    return node_list

def check_consensus():
    required_nodes = ['/node1', '/node2', '/node3'] 
    container_name = 'containerA'  

    rospy.init_node('consensus_check', anonymous=True)
    pub = rospy.Publisher('/consensus_status', String, queue_size=10)

    # Check if all required nodes are running
    running_nodes = get_running_nodes(container_name)
    all_nodes_running = all(node in running_nodes for node in required_nodes)

    if not all_nodes_running:
        rospy.logwarn("Not all required nodes are running.")
        pub.publish("Not all required nodes are running.")
        return

    # Check if each node can see every other node
    all_nodes_can_see_each_other = True
    for node in required_nodes:
        command = f"rosnode info {node}"
        node_info = exec_command_in_container(container_name, command)
        for other_node in required_nodes:
            if other_node != node and other_node not in node_info:
                rospy.logwarn(f"{node} cannot see {other_node}.")
                all_nodes_can_see_each_other = False

    if all_nodes_can_see_each_other:
        rospy.loginfo("All nodes can see each other.")
        pub.publish("All nodes can see each other.")
    else:
        rospy.logwarn("Not all nodes can see each other.")
        pub.publish("Not all nodes can see each other.")

if __name__ == "__main__":
    while not rospy.is_shutdown():
        check_consensus()
        rospy.sleep(30)  # Interval between health checks
