import rospy
import rosnode
from std_msgs.msg import String

def check_node_topic_connections():
    rospy.init_node('node_topic_connections_check', anonymous=True)
    nodes = rosnode.get_node_names()
    node_connections = {}
    
    for node in nodes:
        try:
            publishers, subscribers, services = rospy.get_published_topics(node)
            node_connections[node] = {
                'publishers': publishers,
                'subscribers': subscribers
            }
        except Exception as e:
            rospy.logwarn(f"Failed to get topic connections for {node}: {e}")

    rospy.loginfo(f"Node topic connections: {node_connections}")
    pub = rospy.Publisher('/phm_node_topic_connections', String, queue_size=10)
    pub.publish(str(node_connections))
