# mavlink_handler.py
import time
import json
from pymavlink import mavutil

class MavLinkHandler:
    def __init__(self, port, system_id):
        self.connection = mavutil.mavlink_connection(f"udp:localhost:{port}", source_system=system_id)
        self.connection.wait_heartbeat()
        print("Heartbeat received from the system")

    def send_health_vector(self, agent_id, health_vector):
        self.connection.mav.debug_vect_send(
            name=str(agent_id).encode(),
            time_usec=int(time.time() * 1e6),
            x=health_vector[0],
            y=health_vector[1],
            z=health_vector[2]
        )
        print(f"Sent health vector for agent {agent_id}: Companion={health_vector[0]}, Docker={health_vector[1]}, ROS={health_vector[2]}")

    def send_consensus_message(self, reporting_agent_id, consensus_matrix):
        for observed_agent_id, health_vector in consensus_matrix.items():
            consensus_data = {
                "ra": reporting_agent_id,
                "oa": observed_agent_id,
                "hv": health_vector
            }
            consensus_json = json.dumps(consensus_data, separators=(',', ':'))
            self.connection.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                consensus_json.encode()
            )
            print(f"Sent consensus report: {consensus_json}")

    def receive_messages(self, message_type="DEBUG_VECT", timeout=20):
        start_time = time.time()
        messages = []
        while time.time() - start_time < timeout:
            msg = self.connection.recv_match(blocking=True)
            if msg and msg.get_type() == message_type:
                print(f"Received message: {msg}")
                messages.append(msg)
        return messages

    def disconnect(self):
        self.connection.close()
        print("Disconnected from MAVLink")
