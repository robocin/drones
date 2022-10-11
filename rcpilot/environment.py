"""Created by felipe-nunes on 22/09/2022

- Environment values
"""

class Communication:
    # BEWARE: if a new type of connection string is defined here,
    #         a new corresponding if statement must be defined at
    #         drone.py > class Drone > def resolve_connection_string

    # SIMULATION CONSTANTS
    SIMULATION_DEFAULT_PORT = 14540
    SIMULATION_DEFAULT_PROTOCOL = "udp"
    SIMULATION_CONN_STRING = "{}://:{}".format(
        SIMULATION_DEFAULT_PROTOCOL, SIMULATION_DEFAULT_PORT)

    # HARDWARE CONSTANTS
    HARDWARE_DEFAULT_PORT = "/dev/ttyACM1" 
    HARDWARE_DEFAULT_PROTOCOL = "serial"
    HARDWARE_CONN_STRING = "{}://{}".format(
        HARDWARE_DEFAULT_PROTOCOL, HARDWARE_DEFAULT_PORT)


class Arena:
    ARENA_HEIGHT_METERS = 4
    ARENA_MAX_X_METERS = 8
    ARENA_MAX_Y_METERS = 8

    SAFE_NET_LATERAL_DISTANCE_CENTIMETERS = 50
    SAFE_NET_ABOVE_DISTANCE_CENTIMETERS = 50

    SUSPENDED_BASE = "SUSPENDED_BASE"
    GROUND_BASE = "GROUND_BASE"
    HOME_BASE = "HOME_BASE"
    UNKNOWN_BASE = "UNKNOWN_BASE"

    def test():
        print(HARDWARE_CONN_STRING)


class Navigation:
    MAX_SPEED_MS = 1
    MIN_SPEED_MS = 0.2

    SEARCH_RADIUS_METERS = 1
    SEARCH_IDLE_TIME_S = 2
    SEARCH_SQUARE_EDGE_METERS = 2.0
    SEARCH_NED_HEIGHT = -2.0


class Takeoff:
    TIMEOUT_SECONDS = 10
    SPEED_MS = 1
    ALTITUDE_METERS = 10
    SAFE_ALTITUDE_METERS = 0.5
    LARC_ALTITUDE_METERS = 1


class Land:
    TIMEOUT_SECONDS = 5
    SPEED_MS = 1
