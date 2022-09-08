import asyncio
from mavsdk import System 
from abc import ABC
from constants import Mission, Constants

class SafePilot(ABC):
    def __init__(self, mission_type: Mission) -> None:
        self.mission_type = mission_type
        self.system = System()
        self.thread_listener = None


    async def connect(self):
        await self.system.connect(system_address = self.__system_address())
        self.thread_listener = asyncio.ensure_future(self.__print_status_text())


    def __system_address(self):
        return "{}://{}".format(Constants.COMM_DEFAULT_PROTOCOL, Constants.COMM_DEFAULT_PORT)
        

    async def __print_status_text(self):
        try:
            async for status_text in self.system.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return
         

    async def land(self):
        pass


    async def takeoff(self):
        pass


    async def change_flight_mode(self):
        pass


    async def failsafe(self):
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

