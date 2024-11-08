from scripts.docker.docker_health_checks import Run_Docker_Health_Checks
from scripts.ros.ros_checks import Run_Ros_Health_Checks
from scripts.log_manager import log_message, generate_report, compress_report
import time

from scripts.mavlink.health_report_driver import run_communication

def CompanionComputerAliveStatus():
    log_message("SUCCESS: Companion Computer is running aboard agent.")

def main():

    print('start')
    run_communication()

# while (True):
#         # Layer 1 -- Companion Computer 
#         CompanionComputerAliveStatus()
        
#         # Layer 2 -- Docker Container
#         Run_Docker_Health_Checks()

#         # Layer 3 -- ROS Nodes/Enviorment
#         Run_Ros_Health_Checks()

#         #Wait 
#         print("Creating Report | End of Cycle")
#         generate_report()
#         compress_report()
#         time.sleep(10)
        
if __name__ == "__main__":
    main()