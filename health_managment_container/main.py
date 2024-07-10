from scripts.docker.docker_health_checks import Run_Docker_Health_Checks
from scripts.ros.ros_checks import Run_Ros_Health_Checks

def CompanionComputerAliveStatus():
    print("Companion Computer is running aboard agent.")

def main():
    # Layer 1 -- Companion Computer 
    CompanionComputerAliveStatus()
    
    # Layer 2 -- Docker Container
    Run_Docker_Health_Checks()

    # Layer 3 -- ROS Nodes/Enviorment
    Run_Ros_Health_Checks()
    
if __name__ == "__main__":
    main()