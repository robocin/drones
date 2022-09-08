import asyncio
import time
from mavsdk import System 
from abc import ABC
from constants import Mission, Constants
from debugger import STDOUT

class SafePilot(ABC):
    def __init__(self, mission_type: Mission) -> None:
        self.mission_type = mission_type
        self.system = System()
        self.thread_listener = None
        self.epoch = time.time()


    async def connect(self):
        await self.system.connect(system_address = self.__system_address())
        self.thread_listener = asyncio.ensure_future(self.__print_status_text())

        STDOUT.debug(self.__ctime(), self.CONTEXT, "Waiting for drone to connect")
        async for state in self.system.core.connection_state():
            if state.is_connected:
                STDOUT.debug(self.__ctime(), self.CONTEXT, "Connected to drone!")
                break


    def __system_address(self):
        return "{}://{}".format(Constants.COMM_DEFAULT_PROTOCOL, Constants.COMM_DEFAULT_PORT)


    def __ctime(self):
        return time.time() - self.epoch


    async def __print_status_text(self):
        try:
            async for status_text in self.system.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return
         

    async def arm(self):
        STDOUT.debug(self.__ctime(), self.CONTEXT, "Arming")
        await self.system.action.arm()


    async def takeoff(self):
        self.__ensure_safe_takeoff_height()
        STDOUT.debug(self.__ctime(), self.CONTEXT, "Taking off")
        await self.system.action.takeoff()


    def __ensure_safe_takeoff_height():
        raise NotImplementedError


    async def land(self):
        pass


    async def change_flight_mode(self):
        pass


    async def failsafe(self):
        pass


    CONTEXT = "PILOT"


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


    CONTEXT = "ROBOCIN_PILOT"
