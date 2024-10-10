from enum import Enum, auto

# Enum for Critical Levels
class CriticalLevel(Enum):
    LOW = "Low"
    MEDIUM = "Medium"
    HIGH = "High"

# Enum for Check Types
class CheckType(Enum):
    SOFTWARE = "Software"
    HARDWARE = "Hardware"
    NETWORK = "Network"

class HealthLevel(Enum):
    SUCCESSFUL = 0
    COMPANION_COMPUTER = 1
    DOCKER = 2
    ROS = 3
    UNRESPONSIVE = 4
    MULTIPLE_CRITICAL = 5