import time
from scripts.ros.ros_node_consensus import check_consensus
from scripts.ros.ros_topic_traffic import check_topic_subscriptions, monitor_topic_traffic

def Run_Ros_Health_Checks():
    print("ROS: Checking health of ros component.")
    check_consensus()
    check_topic_subscriptions()
    monitor_topic_traffic()


    