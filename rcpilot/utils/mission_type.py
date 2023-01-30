"""Created by felipe-nunes on 22/09/2022
"""


class MissionType:
    NO_MISSION = -1
    CALIBRATION = 0
    MISSION_1 = 1
    MISSION_2 = 2
    MISSION_3 = 3
    MISSION_4 = 4
    TECHNICAL_CHALLENGE = 5
    TESTING = 6

    @staticmethod
    def as_string(mission):
        match mission:
            case -1:
                return
            case 0:
                return "CALIBRATION"
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
            case 6:
                return "SEARCH"
            case 7:
                return "THRUST"
