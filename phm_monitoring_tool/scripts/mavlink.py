import time
import os
from phm_monitoring_tool.config.HealthChecks import HealthLevel
from pymavswarm import MavSwarm
from argparse import ArgumentParser
from concurrent.futures import Future


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("port", type=str, help="port to establish a MAVLink connection over")
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    return parser.parse_args()

def print_message_response_cb(future: Future) -> None:
    responses = future.result()
    for response in responses:
        print(f"Result of {response.message_type} message sent to ({response.target_agent_id}): {response.code}")
    return


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


def broadcast_health_check(mavswarm, agent_id, health_check_report):
    message = f"{agent_id} health_check: {','.join(map(str, health_check_report))}"
    future = mavswarm.send_debug_message("health_check", message)
    future.add_done_callback(print_message_response_cb)


def listen_for_reports(mavswarm, timeout=30):
    start_time = time.time()
    received_reports = {}

    print(f"Listening for reports for {timeout} seconds...")

    while time.time() - start_time < timeout:
        incoming_messages = mavswarm.get_incoming_messages()  # Replace with actual method
        for message in incoming_messages:
            sender_id = message["sender_id"]
            report_data = message["message"].split("health_check: ")[1]
            health_check_report = list(map(int, report_data.split(',')))
            received_reports[sender_id] = health_check_report
            print(f"Received health report from Drone {sender_id}: {health_check_report}")

        time.sleep(0.1)  # Avoid busy-waiting

    return received_reports


def calculate_consensus(received_reports, health_checks):
    """
    Calculate the consensus based on received reports, ensuring that if there are multiple
    levels of failures, the drone is marked as multiple/critical.

    :param received_reports: Dictionary of {drone_id: health_check_report}
    :param health_checks: List of health checks with details about critical levels and health levels
    :return: Consensus report {drone_id: HealthLevel}
    """
    consensus = {}

    # Convert health_checks to a dictionary for easy lookup by check ID
    health_check_map = {check["id"]: check for check in health_checks}

    for drone_id, report in received_reports.items():
        failing_levels = set()  # A set to track distinct failing levels

        for check_id, result in enumerate(report):
            if result == 1:  # If a failure is reported
                check = health_check_map.get(check_id + 1)  # Get the corresponding check by ID (index + 1)
                if check:
                    failing_levels.add(check["health_level"])

        # If no failures, mark as successful
        if not failing_levels:
            consensus[drone_id] = HealthLevel.SUCCESSFUL
        elif len(failing_levels) > 1:
            # If there are multiple distinct failing levels, mark as MULTIPLE_CRITICAL
            consensus[drone_id] = HealthLevel.MULTIPLE_CRITICAL
        else:
            # If all failing checks are at the same level, return that level
            consensus[drone_id] = failing_levels.pop()

    return consensus

def broadcast_consensus(mavswarm, agent_id, consensus):
    consensus_message = f"{agent_id} consensus: {','.join([f'Drone {drone_id}: {status}' for drone_id, status in consensus.items()])}"
    future = mavswarm.send_debug_message("consensus_report", consensus_message)
    future.add_done_callback(print_message_response_cb)


def main():
    # Parse the script arguments
    args = parse_args()

    # Create a new MavSwarm instance and connect
    mavswarm = MavSwarm()
    if not mavswarm.connect(args.port, args.baud):
        print(f"Failed to connect to port {args.port} with baud {args.baud}")
        return

    agent_id = mavswarm.get_agent_id()  # Get the ID of this drone

    # Step 1: Extract the health report from the log file
    log_file = "/src/healthmanagement/logs/health_report.log"
    health_report = extract_last_health_report(log_file)

    if not health_report:
        print(f"No valid health report found for drone {agent_id}. Aborting.")
        mavswarm.disconnect()
        return

    # Step 2: Broadcast the health check report to all other drones
    broadcast_health_check(mavswarm, agent_id, health_report)

    # Step 3: Listen for reports from other drones
    received_reports = listen_for_reports(mavswarm, timeout=30)

    # Step 4: Calculate the consensus based on the received reports
    consensus = calculate_consensus(received_reports)

    # Step 5: Broadcast the consensus back to all drones
    broadcast_consensus(mavswarm, agent_id, consensus)

    # Disconnect from the swarm
    mavswarm.disconnect()
    print("Disconnected from the swarm.")


if __name__ == "__main__":
    main()
