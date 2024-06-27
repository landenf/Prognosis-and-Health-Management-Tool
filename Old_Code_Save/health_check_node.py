#!/usr/bin/python3
import rospy
import rosnode
from std_msgs.msg import String

# NodeChecker class
class NodeChecker:
    def __init__(self, nodes_to_check):
        self.nodes_to_check = nodes_to_check
        self.rate = rospy.Rate(1)  # Check every second

    #Identify 
    #Check
    #Respond

    def check_alive(self):
        while not rospy.is_shutdown():
            node_names = rosnode.get_node_names()
            print(node_names)
            for node in self.nodes_to_check:
                if node in node_names:
                    rospy.loginfo(f"Nodcoree {node} is up and running.")
                else:
                    rospy.logwarn(f"Node {node} is not running.")
            

            self.rate.sleep()

def chatter_callback(message):
        rospy.loginfo(rospy.get_caller_id() + " I recieved %s", message.data)

if __name__ == "__main__":
    rospy.init_node('health_check_node')

    nodes_to_check = ['/test_node']
    checker = NodeChecker(nodes_to_check)

    #rospy. ("chatter", String, chatter_callback)
    checker.check_alive()

