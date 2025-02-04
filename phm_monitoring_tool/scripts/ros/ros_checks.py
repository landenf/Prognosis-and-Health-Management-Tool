from scripts.ros.ros_node_consensus import check_nodes
from scripts.ros.ros_topic_traffic import check_topic_subscriptions, monitor_topic_traffic
import os

def Run_Ros_Health_Checks():
    container_name = os.getenv('CONTAINER_TO_MONITOR')
    sys_id = os.getenv('sys_id')
    print(container_name, sys_id)
    print("ROS: ros checks initiated")
    check_nodes(sys_id, container_name)
    check_topic_subscriptions(sys_id, container_name)
    monitor_topic_traffic(sys_id, container_name)


    