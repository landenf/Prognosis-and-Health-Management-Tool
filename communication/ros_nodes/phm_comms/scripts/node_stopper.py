#!/usr/bin/env python3
import rospy
import rosnode

from std_msgs.msg import Int32

from phm_comms import decode_msg, decode_report

#Globals may need to change:

_ELEMET_TO_CHECK_FOR_FAILURE = 4
_NODE_TO_KILL = '/agent_1/node_1'
global_detect_failure = False

def report_received_callback(msg):
    
    #decode the message int into 2 16bit components
    id, report = decode_msg(msg.data)

    report_list = decode_report(report,16)
    

    for i in range(6):
        report_list.pop(0)

    print(report_list)


    if report_list[_ELEMET_TO_CHECK_FOR_FAILURE] == 0:
        global global_detect_failure
        global_detect_failure = True


if __name__ == "__main__":

    rospy.init_node("node_stopper")
    rospy.Subscriber('/radio_traffic', Int32, report_received_callback, queue_size=30)

    #Set main loop period 10hz
    rate = rospy.Rate(10)

    #Kill the node
    rosnode.kill_nodes([_NODE_TO_KILL])
    rospy.logwarn(f"Successfully killed {_NODE_TO_KILL}")

    start = rospy.get_time()

    while not global_detect_failure:
        
        rate.sleep()

    finish = rospy.get_time()


    print(finish-start)