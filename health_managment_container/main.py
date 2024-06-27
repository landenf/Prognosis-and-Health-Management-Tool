from Demo.health_managment_container.HealthCheckScripts.Docker_Checks import Run_Docker_Health_Checks

def CompanionComputerAliveStatus():
    print("Companion Computer Is Running aboard agent.")

def main():
    CompanionComputerAliveStatus()
    Run_Docker_Health_Checks()

if __name__ == "__main__":
    main()