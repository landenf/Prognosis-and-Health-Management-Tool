import json
import time
from pymavlink import mavutil
from concurrent.futures import Future

# Class for handling MAVLink communication
class MavLinkHandler:
    def __init__(self, port):
        self.connection = mavutil.mavlink_connection(f"udp:localhost:{port}")
        self.connection.wait_heartbeat()
        print("Heartbeat received from the system")


    def send_int_array(self, array, agent_id):
        # Convert the array of integers to floats (since DEBUG_FLOAT_ARRAY expects floats)
        float_array = [float(i) for i in array]
        
        # Send the array using DEBUG_FLOAT_ARRAY
        self.connection.mav.debug_float_array_send(
            time_boot_ms=int(time.time() * 1000),  # Timestamp in milliseconds
            name=agent_id.encode(),                    
            array=float_array                      
        )
        print(f"Sent array: {array} as floats: {float_array}")

    def send_consensus_message(self, consensus_array):
        consensus_json = json.dumps(consensus_array)

        self.connection.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO, 
            consensus_json.encode()  # Encode JSON string as bytes
        )
        print(f"Sent consensus message: {consensus_json}")

    def receive_messages(self, message_type="DEBUG_FLOAT_ARRAY", timeout=30):
        start_time = time.time()
        messages = []

        while time.time() - start_time < timeout:
            msg = self.connection.recv_match(type=message_type, blocking=True)
            if msg:
                messages.append(msg)
        return messages

    def disconnect(self):
        self.connection.close()
        print("Disconnected from MAVLink")

# Callback for printing message response
def print_message_response_cb(future: Future) -> None:
    responses = future.result()
    for response in responses:
        print(f"Result of {response.message_type} message: {response.code}")
