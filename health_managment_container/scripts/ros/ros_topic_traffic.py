import os
import time
import docker

# Initialize Docker client
client = docker.from_env()

def get_running_nodes(container_name):
    command = "rosnode list"
    return exec_command_in_container(container_name, command).split()

def get_node_info(container_name, node):
    command = f"rosnode info {node}"
    return exec_command_in_container(container_name, command)

def get_topic_list(container_name):
    command = "rostopic list"
    return exec_command_in_container(container_name, command).split()

def get_topic_info(container_name, topic):
    command = f"rostopic info {topic}"
    return exec_command_in_container(container_name, command)

def exec_command_in_container(container_name, command):
    container = client.containers.get(container_name)
    result = container.exec_run(command)
    return result.output.decode('utf-8').strip()

def monitor_traffic(container_name, topic):
    command = f"rostopic hz {topic}"
    return exec_command_in_container(container_name, command)


def check_topic_subscriptions():
    container_name = os.getenv('CONTAINER_TO_MONITOR')
    required_topics = ['/topic1', '/topic2', '/topic3']

    topic_list = get_topic_list(container_name)
    all_topics_present = all(topic in topic_list for topic in required_topics)

    if not all_topics_present:
        missing_topics = [topic for topic in required_topics if topic not in topic_list]
        print(f"WARNING: Missing required topics: {', '.join(missing_topics)}")
        return False

    print(f"All required topics are present: {', '.join(required_topics)}")

    all_subscriptions_correct = True
    for topic in required_topics:
        topic_info = get_topic_info(container_name, topic)
        if container_name not in topic_info['Subscribers']:
            print(f"WARNING: Container '{container_name}' is not subscribed to topic: {topic}")
            all_subscriptions_correct = False

    return all_subscriptions_correct


def monitor_topic_traffic():
    container_name = os.getenv('CONTAINER_TO_MONITOR')
    required_topics = ['/topic1', '/topic2', '/topic3']

    for topic in required_topics:
        traffic_data = monitor_traffic(container_name, topic)
        if "average rate:" not in traffic_data:
            print(f"WARNING: No traffic detected on topic: {topic}")
        else:
            print(f"Traffic data for {topic}: {traffic_data}")
 