from scripts.docker.docker_health_checks import RunDockerHealthChecks

def CompanionComputerAliveStatus():
    print("Companion Computer is running aboard agent.")

def main():
    # Layer 1 -- Companion Computer 
    CompanionComputerAliveStatus()
    
    # Layer 2 -- Docker Container
    RunDockerHealthChecks()

    # Layer 3 -- ROS Nodes/Enviorment

if __name__ == "__main__":
    main()