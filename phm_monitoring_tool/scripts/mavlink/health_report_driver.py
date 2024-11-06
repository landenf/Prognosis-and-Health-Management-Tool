import time
import os
import json
from mavlink_communication import MavLinkHandler
from phm_monitoring_tool.config.HealthChecks import HealthLevel, health_checks

# Function to extract the last health report
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

    if last_health_report:
        print(f"Extracted last health report: {last_health_report}")
    else:
        print("No health report found in the log file.")

    return last_health_report

# Function to broadcast health check report
def broadcast_health_check(handler, agent_id, health_check_report):
    handler.send_int_array(health_check_report, name=f"Drone_{agent_id}")
    print(f"Broadcasted health check report from Drone {agent_id}: {health_check_report}")

# Function to listen for health reports
def listen_for_reports(handler, timeout=30):
    print(f"Listening for reports for {timeout} seconds...")
    received_reports = {}
    messages = handler.receive_messages(timeout=timeout)

    for message in messages:
        sender_id = message.name.decode()  # Assuming message name includes the sender ID
        health_check_report = message.array  
        health_check_report = list(map(int, health_check_report)) 
        received_reports[sender_id] = health_check_report
        print(f"Received health report from {sender_id}: {health_check_report}")

    return received_reports

def calculate_consensus(received_reports, health_checks):
    consensus = {}
    health_check_map = {check["id"]: check for check in health_checks}

    for drone_id, report in received_reports.items():
        failing_levels = set()
        for check_id, result in enumerate(report):
            if result == 1:
                check = health_check_map.get(check_id + 1)
                if check:
                    failing_levels.add(check["health_level"])

        if not failing_levels:
            consensus[drone_id] = HealthLevel.SUCCESSFUL
        elif len(failing_levels) > 1:
            consensus[drone_id] = HealthLevel.MULTIPLE_CRITICAL
        else:
            consensus[drone_id] = failing_levels.pop()

    return consensus

def main():
    port = os.environ.get("MAVLINK_PORT", "14560")
    handler = MavLinkHandler(port)
    agent_id = handler.connection.target_system

    log_file = "/src/healthmanagement/logs/health_report.log"
    health_report = extract_last_health_report(log_file)

    if not health_report:
        print(f"No valid health report found for drone {agent_id}. Aborting.")
        handler.disconnect()
        return

    broadcast_health_check(handler, agent_id, health_report)

    received_reports = listen_for_reports(handler)
    
    consensus = calculate_consensus(received_reports, health_checks)
    
    consensus_message = json.dumps({drone_id: status for drone_id, status in consensus.items()})
    handler.send_consensus_message(consensus_message)
    
    handler.disconnect()

if __name__ == "__main__":
    main()