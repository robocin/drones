
class ConnectionType:
    SIMULATION = 0
    HARDWARE = 1

    @staticmethod
    def as_string(mission):
        match mission:
            case 0:
                return "SIMULATION"
            case 1:
                return "HARDWARE"
