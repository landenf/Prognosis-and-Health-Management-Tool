#!/usr/bin/python3
import rospy
from std_msgs.msg import String

class Node2:
    def __init__(self):
        rospy.init_node('node_2')

        # Publisher for topic_2
        self.pub_topic_2 = rospy.Publisher('topic_2', String, queue_size=10)

        # Subscribers to the other topics
        rospy.Subscriber('topic_1', String, self.callback_topic_1)
        rospy.Subscriber('topic_3', String, self.callback_topic_3)

        self.rate = rospy.Rate(1)  # 1 Hz for publishing

    def callback_topic_1(self, data):
        rospy.loginfo(f"Node 2 heard from topic_1: {data.data}")

    def callback_topic_3(self, data):
        rospy.loginfo(f"Node 2 heard from topic_3: {data.data}")

    def publish_to_topic_2(self):
        msg = "Message from node 2 to topic_2"
        rospy.loginfo(f"Publishing to topic_2: {msg}")
        self.pub_topic_2.publish(msg)

    def run(self):
        rospy.loginfo("Node 2 starting...")
        while not rospy.is_shutdown():
            self.publish_to_topic_2()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = Node2()
        node.run()
    except rospy.ROSInterruptException:
        pass
