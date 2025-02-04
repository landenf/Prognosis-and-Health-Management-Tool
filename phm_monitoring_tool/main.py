from scripts.docker.docker_health_checks import Run_Docker_Health_Checks
from scripts.ros.ros_checks import Run_Ros_Health_Checks
from scripts.log_manager import log_message, generate_report, compress_report
import time


def CompanionComputerAliveStatus():
    log_message("SUCCESS: (1) Companion Computer is running aboard agent.")

def main():
    
    while (True):
        # Layer 1 -- Companion Computer 
        CompanionComputerAliveStatus()
        
        # Layer 2 -- Docker Container
        docker_successs = Run_Docker_Health_Checks()

        # Layer 3 -- ROS Nodes/Enviorment only continue if docker is successful
        if(docker_successs):
            Run_Ros_Health_Checks()

        #Wait 
        print("Creating Report | End of Cycle")
        generate_report()
        compress_report()
        time.sleep(20)
        
if __name__ == "__main__":
    main()