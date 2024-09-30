from scripts.docker.docker_health_checks import Run_Docker_Health_Checks
from scripts.ros.ros_checks import Run_Ros_Health_Checks
import time

def CompanionComputerAliveStatus():
    print("START: Companion Computer is running aboard agent.")

def main():
    
    while (True):
        # Layer 1 -- Companion Computer 
        CompanionComputerAliveStatus()
        
        # Layer 2 -- Docker Container
        Run_Docker_Health_Checks()

        # Layer 3 -- ROS Nodes/Enviorment
        Run_Ros_Health_Checks()

        #Wait 
        print("Waiting")
        time.sleep(10)
        
if __name__ == "__main__":
    main()