#!/usr/bin/python3

import rospy
from std_msgs.msg import String

class TalkerAndListenerNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('talker')

        # Publishers
        self.pub_topic_1 = rospy.Publisher('topic_1', String, queue_size=10)
        self.pub_topic_2 = rospy.Publisher('topic_2', String, queue_size=10)
        self.pub_topic_3 = rospy.Publisher('topic_3', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('topic_2', String, self.callback_topic_2)

        # Rate for the publishers
        self.rate_1hz = rospy.Rate(1)  # 1 Hz for topic_1 (every 1 second)
        self.rate_0_5hz = rospy.Rate(0.5)  # 0.5 Hz for topic_2 (every 2 seconds)

    def callback_topic_2(self, data):
        rospy.loginfo(f"Node heard from topic_2: {data.data}")

    def publish_to_topic_1(self):
        msg = "Message from node publishing to topic_1"
        rospy.loginfo(f"Publishing to topic_1: {msg}")
        self.pub_topic_1.publish(msg)

    def publish_to_topic_2(self):
        msg = "Message from node publishing to topic_2"
        rospy.loginfo(f"Publishing to topic_2: {msg}")
        self.pub_topic_2.publish(msg)

    def publish_to_topic_3(self):
        msg = "Message from node publishing to topic_3"
        rospy.loginfo(f"Publishing to topic_3: {msg}")
        self.pub_topic_3.publish(msg)

    def run(self):
        rospy.loginfo("Starting node...")
        while not rospy.is_shutdown():
            self.publish_to_topic_1()  # Publish to topic_1 every second
            self.rate_1hz.sleep()

            self.publish_to_topic_2()  # Publish to topic_2 every 2 seconds
            self.rate_0_5hz.sleep()

            self.publish_to_topic_3()  # Publish to topic_2 every 2 seconds
            self.rate_1hz.sleep()

if __name__ == "__main__":
    try:
        node = TalkerAndListenerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
