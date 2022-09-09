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
    SEARCH = 6

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

class MessageType:
    """Message types for debugger
    """
    WARNING = 0
    ERROR = 1
    LOG = 2

    @staticmethod
    def as_string(message):
        match message:
            case 0:
                return "WARNING"
            case 1:
                return "ERROR"
            case 2:
                return "LOG"
            case _:
                return message
            
 

class Constants:
    """Constant values for reference
    """
    ARENA_HEIGHT_METERS = 4
    ARENA_MAX_X_METERS = 8
    ARENA_MAX_Y_METERS = 8

    COMM_DEFAULT_PORT = 14540
    COMM_DEFAULT_PROTOCOL = "udp"
    COMM_CONN_STRING = "{}://:{}".format(COMM_DEFAULT_PROTOCOL, COMM_DEFAULT_PORT)

    DRONE_MAX_SPEED_MS = 1
    DRONE_MIN_SPEED_MS = 0.2
    DRONE_TAKEOFF_SPEED_MS = 1
    DRONE_LAND_SPEED_MS = 1
    DRONE_TAKEOFF_HEIGHT_METERS = 1
    DRONE_SAFE_TAKEOFF_HEIGHT_METERS = 0.2
    DRONE_SEARCH_RADIUS_METERS = 1
    DRONE_SEARCH_IDLE_TIME_S = 2
    DRONE_SEARCH_SQUARE_EDGE_METERS = 2.0
    DRONE_SEARCH_NED_HEIGHT = -2.0

    SAFE_NET_LATERAL_DISTANCE_CENTIMETERS = 50
    SAFE_NET_ABOVE_DISTANCE_CENTIMETERS = 50
    SAFE_OFFBOARD_TIMEOUT_SECONDS = 10

