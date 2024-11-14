# main_script.py
import time
import os
import threading
from scripts.mavlink.mavlink_handler import MavLinkHandler

# This flag will control when to stop the threads
stopped = False

def extract_last_health_report(log_file_path):
    try:
        with open(log_file_path, 'r') as log_file:
            # Read all lines and only keep the last one
            last_line = log_file.readlines()[-1].strip()
            
            # Check if "Compressed:" is in the last line
            if "Compressed:" in last_line:
                # Extract the numbers after "Compressed:" and convert them to integers
                return list(map(int, last_line.split("Compressed:")[1].strip().split(',')))
                
    except FileNotFoundError:
        print(f"Log file not found: {log_file_path}")
    except Exception as e:
        print(f"Error reading last health report: {e}")
    
    return None

def listen_for_reports(handler, received_reports):
    global stopped
    while not stopped:  # Check if the stop flag is set
        messages = handler.receive_messages(timeout=5)  # Adjust timeout as needed
        for message in messages:
            sender_id = message.name
            health_check_report = [message.x, message.y, message.z]
            received_reports[sender_id] = health_check_report
            print(f"Received health vector from agent {sender_id}: {health_check_report}")
        time.sleep(1)

def broadcast_summary(handler, log_file_path, agent_id, received_reports):
    global stopped
    while not stopped:  # Check if the stop flag is set
        health_report = extract_last_health_report(log_file_path)
        if health_report:
            health_vector = [
                1 if health_report[0] > 0 else 0,
                1 if health_report[1] > 0 else 0,
                1 if health_report[2] > 0 else 0
            ]
            handler.send_health_vector(agent_id, health_vector)
            #received_reports[agent_id] = health_vector
            handler.send_consensus_message(agent_id, received_reports)
        time.sleep(5)

def run_communication(duration=60):
    global stopped
    port = os.environ.get("MAVLINK_PORT", "14560")
    system_id = int(os.environ.get("TARGET_SYSTEM_ID", "1"))
    handler = MavLinkHandler(port, system_id)
    agent_id = port[3] #handler.connection.target_system
    log_file_path = "/src/logs/summary_report.txt"

    # Shared dictionary for received reports
    received_reports = {}

    # Start listening and broadcasting threads
    listen_thread = threading.Thread(target=listen_for_reports, args=(handler, received_reports))
    broadcast_thread = threading.Thread(target=broadcast_summary, args=(handler, log_file_path, agent_id, received_reports))
    
    listen_thread.start()
    broadcast_thread.start()

    # Let the threads run for a specified duration (e.g., 60 seconds)
    time.sleep(duration)

    # Signal threads to stop
    stopped = True

    # Wait for both threads to complete
    listen_thread.join()
    broadcast_thread.join()

    handler.disconnect()
    print("Communication run completed and stopped.")


#TODO: 1) fix read in from summary 2) have just broadcast and listen setup corretly and disconnect correctly 3) loop in every time checks run from main
