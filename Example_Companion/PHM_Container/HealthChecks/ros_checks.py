import time
from ros_topic_connections import check_node_topic_connections
from ros_topic_traffic import check_topic_traffic
from ros_node_consensus import check_consensus

if __name__ == "__main__":
    while True:
        check_node_topic_connections()
        check_topic_traffic()
        check_consensus()
        time.sleep(30)  # Interval between health checks
