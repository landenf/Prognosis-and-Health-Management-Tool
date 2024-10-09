from scripts.ros.ros_node_consensus import check_nodes
from scripts.ros.ros_topic_traffic import check_topic_subscriptions, monitor_topic_traffic

def Run_Ros_Health_Checks():
    print("ROS: ros checks initiated")
    check_nodes()
    check_topic_subscriptions()
    monitor_topic_traffic()


    