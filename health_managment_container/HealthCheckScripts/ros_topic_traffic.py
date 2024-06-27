import rospy
from rosgraph_msgs.msg import Log

def log_callback(data):
    rospy.loginfo(f"Log message: {data.msg}")

def check_topic_traffic():
    rospy.init_node('topic_traffic_check', anonymous=True)
    rospy.Subscriber('/rosout', Log, log_callback)
    rospy.spin()
