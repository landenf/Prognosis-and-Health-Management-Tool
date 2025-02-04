#!/usr/bin/python3
import rospy
from std_msgs.msg import String

class Node1:
    def __init__(self):
        rospy.init_node('node_1')

        # Publisher for topic_1
        self.pub_topic_1 = rospy.Publisher('topic_1', String, queue_size=10)

        # Subscribers to the other topics
        rospy.Subscriber('topic_2', String, self.callback_topic_2)
        rospy.Subscriber('topic_3', String, self.callback_topic_3)

        self.rate = rospy.Rate(1)  # 1 Hz for publishing

    def callback_topic_2(self, data):
        rospy.loginfo(f"Node 1 heard from topic_2: {data.data}")

    def callback_topic_3(self, data):
        rospy.loginfo(f"Node 1 heard from topic_3: {data.data}")

    def publish_to_topic_1(self):
        msg = "Message from node 1 to topic_1"
        rospy.loginfo(f"Publishing to topic_1: {msg}")
        self.pub_topic_1.publish(msg)

    def run(self):
        rospy.loginfo("Node 1 starting...")
        while not rospy.is_shutdown():
            self.publish_to_topic_1()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = Node1()
        node.run()
    except rospy.ROSInterruptException:
        pass
