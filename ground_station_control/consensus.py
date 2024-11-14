# consensus.py
import time
import json
import pandas as pd
from pymavlink import mavutil

# Initialize the MAVLink connection
ground_control = mavutil.mavlink_connection('udp:localhost:14590', source_system=4)
ground_control.wait_heartbeat()
print("Heartbeat received from MAVLink system.")

# Initialize an empty DataFrame with dtype=object for flexible data storage
consensus_matrix = pd.DataFrame(dtype=object)

def update_consensus_matrix(reporting_agent, observed_agent, health_vector):
    global consensus_matrix

    # Add the agents to the DataFrame if they are new
    if reporting_agent not in consensus_matrix.index:
        consensus_matrix = consensus_matrix.reindex(index=consensus_matrix.index.union([reporting_agent]), fill_value=None)
    if observed_agent not in consensus_matrix.columns:
        consensus_matrix = consensus_matrix.reindex(columns=consensus_matrix.columns.union([observed_agent]), fill_value=None)

    # Convert the health vector to a string and store it in the DataFrame
    consensus_matrix.loc[reporting_agent, observed_agent] = str(health_vector)

    # Print the updated consensus matrix with lines
    print("\nCurrent Consensus Matrix:")
    print(consensus_matrix.to_string())

def listen_for_consensus():
    print("Listening for Consensus messages...")
    while True:
        # Wait for the next MAVLink message
        msg = ground_control.recv_match(blocking=True)
        
        if msg and msg.get_type() == "STATUSTEXT":  # Assuming `STATUSTEXT` for consensus message
            consensus_json = msg.text
            try:
                # Parse individual consensus report from JSON
                consensus_data = json.loads(consensus_json)
                
                # Extract fields
                reporting_agent = consensus_data["ra"]
                observed_agent = consensus_data["oa"]
                health_vector = consensus_data["hv"]
                
                # Update consensus matrix
                update_consensus_matrix(reporting_agent, observed_agent, health_vector)
                
            except json.JSONDecodeError:
                print("Failed to decode consensus JSON:", consensus_json)

# Start listening for consensus messages
listen_for_consensus()
