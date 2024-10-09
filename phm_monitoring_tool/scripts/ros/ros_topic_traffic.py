import os
import time
import docker
from scripts.log_manager import log_message

client = docker.from_env()

def get_running_nodes(container_name):
    command = "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode list'"
    return exec_command_in_container(container_name, command).split()

def get_node_info(container_name, node):
    command = f"rosnode info {node}"
    return exec_command_in_container(container_name, command)

def get_topic_list(container_name):
    command = "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rostopic list'"
    result = exec_command_in_container(container_name, command).split()
    return result

def get_topic_info(container_name, topic):
    command = f"rostopic info {topic}"
    return exec_command_in_container(container_name, command)

def exec_command_in_container(container_name, command):
    container = client.containers.get(container_name)
    result = container.exec_run(command)
    return result.output.decode('utf-8').strip()

def monitor_traffic(container_name, topic, timeout=5):
    command = f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && timeout {timeout} rostopic hz {topic} | head -n 10'"
    result = exec_command_in_container(container_name, command)
    return result 

def check_topic_subscriptions():
    container_name = os.getenv('CONTAINER_TO_MONITOR')
    required_topics = ['/topic_1','/topic_2', '/topic_3']

    topic_list = get_topic_list(container_name)

    
    all_topics_present = all(topic in topic_list for topic in required_topics)

    if not all_topics_present:
        missing_topics = [topic for topic in required_topics if topic not in topic_list]
        log_message(f"FAILURE: Missing required topics: {', '.join(missing_topics)}")
        return False

    log_message(f"SUCCESS: All required topics are present: {', '.join(required_topics)}")

    all_subscriptions_correct = True

    return all_subscriptions_correct



def monitor_topic_traffic():
    container_name = os.getenv('CONTAINER_TO_MONITOR')
    required_topics = ['/topic_1', '/topic_2', '/topic_3']

    for topic in required_topics:
        traffic_data = monitor_traffic(container_name, topic.lstrip('/'))
        if "average rate:" not in traffic_data:
            log_message(f"FAILURE: No traffic detected on topic: {topic}")
        else:
            lines = traffic_data.splitlines()[:5]  # Get the first 5 lines
            average_rate = None
            for line in lines:
                if 'average rate:' in line:
                    average_rate = line.split('average rate:')[1].strip()
                    break
            if average_rate:
                log_message(f"Average rate for {topic}: {average_rate}")