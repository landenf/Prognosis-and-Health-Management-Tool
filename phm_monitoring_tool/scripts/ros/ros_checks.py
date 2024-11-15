from scripts.ros.ros_node_consensus import check_nodes
from scripts.ros.ros_topic_traffic import check_topic_subscriptions, monitor_topic_traffic

_sys_id = os.getenv('sys_id')

def Run_Ros_Health_Checks(sys_id):
    print("ROS: ros checks initiated")
    check_nodes(sys_id)
    check_topic_subscriptions(sys_id)
    monitor_topic_traffic(sys_id)


    