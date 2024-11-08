import time
import os
import json
from pymavlink import mavutil

# Class for handling MAVLink communication
class MavLinkHandler:
    def __init__(self, port):
        system_id = int(os.environ.get("TARGET_SYSTEM_ID", "1"))  # Default to 1 if not set
        self.connection = mavutil.mavlink_connection(f"udp:localhost:{port}", source_system=system_id)        
        self.connection.wait_heartbeat()
        print("Heartbeat received from the system")

    def send_health_vector(self, agent_id, health_vector):
        # Send health vector using DEBUG_VECT where x = companion, y = docker, z = ROS
        self.connection.mav.debug_vect_send(
            name=str(agent_id).encode(),
            time_usec=int(time.time() * 1e6),
            x=health_vector[0],
            y=health_vector[1],
            z=health_vector[2]
        )
        print(f"Sent health vector for agent {agent_id}: Companion={health_vector[0]}, Docker={health_vector[1]}, ROS={health_vector[2]}")

    def send_consensus_message(self, consensus_matrix):
        consensus_json = json.dumps(consensus_matrix)
        self.connection.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            consensus_json.encode()  # Encode JSON string as bytes
        )
        print(f"Broadcasted consensus matrix: {consensus_json}")

    def receive_messages(self, message_type="DEBUG_VECT", timeout=20):
        start_time = time.time()
        messages = []
        while time.time() - start_time < timeout:
            msg = self.connection.recv_match(blocking=True)
            if(msg.get_type() != 'GLOBAL_POSITION_INT' and msg.get_type() != 'LOCAL_POSITION_NED'):
                print(f"Received message: {msg}")  # Print 
            if msg and msg.get_type() == message_type:
                messages.append(msg)
        return messages

    def disconnect(self):
        self.connection.close()
        print("Disconnected from MAVLink")

# Function to extract the last health report from the log file
def extract_last_health_report(log_file_path):
    if not os.path.exists(log_file_path):
        print(f"Log file not found: {log_file_path}")
        return None

    last_health_report = None
    with open(log_file_path, 'r') as log_file:
        lines = log_file.readlines()
        for line in reversed(lines):
            if "Compressed:" in line:
                compressed_data = line.split("Compressed:")[1].strip()
                last_health_report = list(map(int, compressed_data.split(',')))
                break
    return last_health_report

# Function to broadcast health vector report
def broadcast_health_check(handler, agent_id, health_report):
    # Set health levels for companion, docker, ROS
    health_vector = [
        1 if health_report[0] > 0 else 0,  # Companion
        1 if health_report[1] > 0 else 0,  # Docker
        1 if health_report[2] > 0 else 0   # ROS
    ]
    handler.send_health_vector(agent_id, health_vector)

# Function to listen and build consensus matrix
def listen_for_reports(handler, agent_id, timeout=20):
    print(f"Listening for health vectors for {timeout} seconds...")
    received_reports = {}
    messages = handler.receive_messages(timeout=timeout)

    for message in messages:
        sender_id = message.name
        health_check_report = [message.x, message.y, message.z]
        received_reports[sender_id] = health_check_report
        print(f"Received health vector from agent {sender_id}: {health_check_report}")

    # Build consensus matrix
    consensus_matrix = {}
    for drone_id, report in received_reports.items():
        consensus_matrix[drone_id] = {
            "Companion": report[0],
            "Docker": report[1],
            "ROS": report[2]
        }
    print(consensus_matrix)

    return consensus_matrix

def run_communication():
    port = os.environ.get("MAVLINK_PORT", "14560")
    handler = MavLinkHandler(port)
    agent_id = handler.connection.target_system

    log_file = "/src/logs/summary_report.txt"
    health_report = extract_last_health_report(log_file)

    if not health_report:
        print(f"No valid health report found for drone {agent_id}. Aborting.")
        handler.disconnect()
        return

    if port == '14560':
        broadcast_health_check(handler, agent_id, health_report)
        received_reports = listen_for_reports(handler, agent_id)
        
        # Send consensus matrix as a broadcast message
        handler.send_consensus_message(received_reports)
    else:
        while True:
            time.sleep(5)
            broadcast_health_check(handler, agent_id, health_report)

    handler.disconnect()
