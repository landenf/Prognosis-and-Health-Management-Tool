from pymavlink import mavutil

# Initialize the MAVLink connection
ground_control = mavutil.mavlink_connection('udp:localhost:14550')

# Dictionary to hold consensus data
consensus_data = {}

def update_consensus(agent_id, status_list):
    # Store or update the consensus data for the agent
    consensus_data[agent_id] = status_list

    # Print the 2D array in the required format
    print_consensus_array()

def print_consensus_array():
    # Build and print the 2D array from the consensus_data
    consensus_array = [[f"Agent {agent_id}: {status}" for agent_id, status in consensus_data.items()]]
    for row in consensus_array:
        print(row)

def listen_for_consensus():
    print("Listening for Consensus messages...")
    while True:
        # Wait for the next MAVLink message
        msg = ground_control.recv_match(type='CONSENSUS', blocking=True)
        if not msg:
            continue

        # Parse the consensus message
        agent_id = msg.agent_id  # Assuming MAVLink message has 'agent_id' field
        statuses = msg.statuses  # Assuming 'statuses' is a list of status values in the message

        # Update consensus data with the new message
        update_consensus(agent_id, statuses)

listen_for_consensus()
