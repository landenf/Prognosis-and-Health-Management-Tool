#!/usr/bin/env python3
import rospy
import os

from std_msgs.msg import String, Bool, Float32, Int32

#from phm_monitoring_tool.config.HealthChecks import HealthLevel

#Report bits
_NUMBER_OF_ID_BITS = 16
_NUMBER_OF_REPORT_BITS = 16

def parse_args():
    parser = ArgumentParser()
    parser.add_argument("port", type=str, help="port to establish a MAVLink connection over")
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    return parser.parse_args()

def extract_last_health_report(log_file_path):
    """
    Extract the last health check report from the log file.
    
    :param log_file_path: Path to the log file
    :return: List of 0s and 1s representing the last health check report, or None if no report is found
    """
    if not os.path.exists(log_file_path):
        print(f"Log file not found: {log_file_path}")
        return None
    
    last_health_report = None

    with open(log_file_path, 'r') as log_file:
        lines = log_file.readlines()

        # Find the most recent health report in the log file
        for line in reversed(lines):
            if "Compressed:" in line:
                # Extract the report (everything after "Compressed:")
                compressed_data = line.split("Compressed:")[1].strip()
                # Convert the report into a list of integers
                last_health_report = list(map(int, compressed_data.split(',')))
                break

    if last_health_report:
        print(f"Extracted last health report: {last_health_report}")
    else:
        print("No health report found in the log file.")

    return last_health_report


def encode_msg(sys_id, report):
    """
    Encodes two 16-bit integers into a single 32-bit integer.

    :param sys_id: unsigned 16 bit int representing the sysid
    :param report: unsigned 16 bit int representing the encoded report
    
    """
    if not (0 <= sys_id < 2**16) or not (0 <= report < 2**16):
        raise ValueError("Input values must be 16-bit integers (0 to 65535).")
    return (sys_id << 16) | report


def decode_msg(encoded_int):
    """
    Decodes a 32-bit integer into two 16-bit integers.
    """
    int1 = (encoded_int >> 16) & 0xFFFF  # Extract the upper 16 bits
    int2 = encoded_int & 0xFFFF          # Extract the lower 16 bits
    return int1, int2


def encode_report(bit_list):
    """
    Encodes the health report as an unsigned int

    :param bit_list: list of 1's and 0's
    :return: unsigned int

    """
    result = 0

    for bit in bit_list:
        result = (result << 1) | bit
    return result


def decode_report(number, length):
    """
    Decodes an unsigned integer to a list of 1's and 0's.
    """
    bit_list = []
    for _ in range(length):
        bit_list.append(number & 1)
        number >>= 1
    bit_list.reverse()  # Reverse to match original order
    return bit_list


def report_received_callback(msg):
    
    #decode the message int into 2 16bit components
    id, report = decode_msg(msg.data)

    print(f"report {report} from agent {id}")

# def broadcast_health_check(mavswarm, agent_id, health_check_report):
#     message = f"{agent_id} health_check: {','.join(map(str, health_check_report))}"
#     future = mavswarm.send_debug_message("health_check", message)
#     future.add_done_callback(print_message_response_cb)

# def listen_for_reports(mavswarm, timeout=30):
#     start_time = time.time()
#     received_reports = {}

#     print(f"Listening for reports for {timeout} seconds...")

#     while time.time() - start_time < timeout:
#         incoming_messages = mavswarm.get_incoming_messages()  # TODO Replace with actual method
#         for message in incoming_messages:
#             sender_id = message["sender_id"]
#             report_data = message["message"].split("health_check: ")[1]
#             health_check_report = list(map(int, report_data.split(',')))
#             received_reports[sender_id] = health_check_report
#             print(f"Received health report from Drone {sender_id}: {health_check_report}")
 
#         time.sleep(0.1)

#     return received_reports

if __name__ == "__main__":

    rospy.init_node("phm_comms")

    #Set the agent's ID
    AgentID = rospy.get_param('SystemID', 1)

    #ROS subscribers
    rospy.Subscriber("/radio_traffic", Int32, report_received_callback, queue_size=30 )

    #ROS publsihers
    report_publisher = rospy.Publisher("/radio_traffic", Int32, queue_size=30)

    #Set main loop period ~5 sec
    rate = rospy.Rate(0.2)

    #Main loop
    while not rospy.is_shutdown():

        # Step 1: Extract the health report from the log file
        log_file = "/home/phm_logs/summary_report.txt"
        
        # Step 2: Broadcast the health check report to all other agents
        try:
            #Grab the latest health report as a list
            health_report = extract_last_health_report(log_file)

            #Turn that health report list into an intger
            report_int = encode_report(health_report)

            #Combine the health report and ID into a single 32 bit value
            msg_value = encode_msg(AgentID, report_int)

            #Setup the ROS message
            report_msg = Int32()
            report_msg.data = msg_value
            report_publisher.publish(report_msg)

        except: 
            rospy.logwarn(f"No valid health report found for drone {AgentID} ... Aborting.")

        rate.sleep()
        

        # Step 4: Calculate the consensus based on the received reports

        # Step 5: Broadcast the consensus back to all drones
