class Mission:
    """Missions of drone control and LARC event phases
        
    Each mission represents a type of test, event phase or technical challenge
    """
    CALIBRATION = -1
    TESTING = 0
    MISSION_1 = 1
    MISSION_2 = 2
    MISSION_3 = 3
    MISSION_4 = 4
    TECHNICAL_CHALLENGE = 5

    @staticmethod
    def as_string(mission):
        match mission:
            case -1:
                return "CALIBRATION"
            case 0:
                return "TESTING"
            case 1:
                return "MISSION_1"
            case 2:
                return "MISSION_2"
            case 3:
                return "MISSION_3"
            case 4:
                return "MISSION_4"
            case 5: 
                return "TECHNICAL_CHALLENGE"


class Constants:
    """Constant values for reference
    """
    ARENA_HEIGHT_METERS = 4
    ARENA_MAX_X_METERS = 8
    ARENA_MAX_Y_METERS = 8

    COMM_DEFAULT_PORT = 14540
    COMM_DEFAULT_PROTOCOL = "udp"

    DRONE_MAX_SPEED_MS = 1
    DRONE_MIN_SPEED_MS = 0.2
    DRONE_TAKEOFF_SPEED_MS = 1
    DRONE_LAND_SPEED_MS = 1
    DRONE_TAKEOFF_HEIGHT_METERS = 1

    SAFE_NET_LATERAL_DISTANCE_CENTIMETERS = 50
    SAFE_NET_ABOVE_DISTANCE_CENTIMETERS = 50
    SAFE_OFFBOARD_TIMEOUT_SECONDS = 10

