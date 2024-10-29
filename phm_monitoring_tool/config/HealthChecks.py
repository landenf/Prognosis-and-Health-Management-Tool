
from phm_monitoring_tool.config.Enums import CheckType, CriticalLevel, HealthLevel


health_checks = [
    {
        "id": 1,
        "name": "Companion Computer",
        "abbreviation": "CC",
        "critical_level": CriticalLevel.HIGH,
        "description": "This check ensures that the base companion computer is alive and tracking back a heartbeat.",
        "check_type": CheckType.HARDWARE,
        "health_level" : HealthLevel.COMPANION_COMPUTER,
        "dependent_checks": [] 
    },
    {
        "id": 2,
        "name": "Monitoring Container Healthy",
        "abbreviation": "MCH",
        "critical_level": CriticalLevel.HIGH,
        "description": "This check ensures the user-created health check on the container is running and marked as healthy.",
        "check_type": CheckType.SOFTWARE,
        "health_level" : HealthLevel.DOCKER,
        "dependent_checks": []
    },
    {
        "id": 3,
        "name": "Present and Executable Files",
        "abbreviation": "PEF",
        "critical_level": CriticalLevel.LOW,
        "description": "This check verifies that all required ROS scripts and executable files are present and have the correct permissions.",
        "check_type": CheckType.SOFTWARE,
        "health_level" : HealthLevel.DOCKER,
        "dependent_checks": [2]  
    },
    {
        "id": 4,
        "name": "Network Check",
        "abbreviation": "NC",
        "critical_level": CriticalLevel.MEDIUM,
        "description": "This check ensures that the PHM tool container and the ROS app container are on the same network and can communicate with each other.",
        "check_type": CheckType.NETWORK,
        "health_level" : HealthLevel.DOCKER,
        "dependent_checks": [2]
    },
    {
        "id": 5,
        "name": "Critical External Hardware Connected",
        "abbreviation": "CEHC",
        "critical_level": CriticalLevel.MEDIUM,
        "description": "This check verifies that any critical external hardware (e.g., sensors or USB devices) required for operation is connected and recognized by the system.",
        "check_type": CheckType.HARDWARE,
        "health_level" : HealthLevel.DOCKER,
        "dependent_checks": [2]
    },
    {
        "id": 6,
        "name": "ROS Node Running",
        "abbreviation": "RNR",
        "critical_level": CriticalLevel.HIGH,
        "description": "This check ensures that all critical ROS nodes are active and agree on the system state.",
        "check_type": CheckType.SOFTWARE,
        "health_level" : HealthLevel.ROS,
        "dependent_checks": []
    },
    {
        "id": 7,
        "name": "Critical Node Topic Connections",
        "abbreviation": "CNTC",
        "critical_level": CriticalLevel.MEDIUM,
        "description": "This check verifies that critical ROS nodes are properly subscribed to and publishing on the required topics.",
        "check_type": CheckType.NETWORK,
        "health_level" : HealthLevel.ROS,
        "dependent_checks": [6]
    },
    {
        "id": 8,
        "name": "Topic Traffic Monitoring",
        "abbreviation": "TTM",
        "critical_level": CriticalLevel.MEDIUM,
        "description": "This check monitors the traffic on key ROS topics to ensure that data is being transmitted and received at the expected rates.",
        "check_type": CheckType.NETWORK,
        "health_level" : HealthLevel.ROS,
        "dependent_checks": [7]  
    }
]
