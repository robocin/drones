from abc import ABC
from constants import Mission

class SafePilot(ABC):
    def __init__(self, mission_type: Mission) -> None:
        self.mission_type = mission_type

    def land(self):
        pass

    def takeoff(self):
        pass

    def change_flight_mode(self):
        pass

    def failsafe(self):
        pass


class RobocinPilot(SafePilot):
    def __init__(self, mission_type: Mission) -> None:
        super().__init__(mission_type)

    def land(self):
        return super().land()

    def takeoff(self):
        return super().takeoff()

    def change_flight_mode(self):
        return super().change_flight_mode()

    def failsafe(self):
        return super().failsafe()

