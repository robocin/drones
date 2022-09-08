import asyncio
import time
from mavsdk import System 
from abc import ABC
from constants import Mission, Constants, MessageType
from debugger import STDOUT


class SafePilot(ABC):
    def __init__(self, mission_type: Mission) -> None:
        self.mission_type = mission_type


    def __ensure_safe_takeoff_height():
        raise NotImplementedError


    CONTEXT = "PILOT"


class RobocinPilot(SafePilot):
    def __init__(self, mission_type: Mission) -> None:
        super().__init__(mission_type)
        self.drone: System = System()
    

    async def start_connection(self):
        STDOUT.debug(self.CONTEXT, "Connecting to {}".format(Constants.COMM_CONN_STRING))
        await self.drone.connect(system_address = Constants.COMM_CONN_STRING)
        self.thread_listener = asyncio.ensure_future(self.__print_status_text())
        
        STDOUT.debug(self.CONTEXT, "Waiting for drone to connect")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                STDOUT.debug(self.CONTEXT, "Connected to drone!")
                break

    
    async def arm(self):
        STDOUT.debug(self.CONTEXT, "Arming")
        await self.drone.action.arm()
        STDOUT.debug(self.CONTEXT, "Drone armed")
        STDOUT.debug(MessageType.WARNING, "Step back motor starting")


    # async def takeoff(self):
    #     self.__ensure_safe_takeoff_height()
    #     STDOUT.debug(self.CONTEXT, "Taking off")
    #     await self.drone.action.takeoff()


    # async def land(self):
    #     await super().land()


    # async def takeoff(self):
    #     await super().takeoff()


    # def __ensure_safe_takeoff_height():
    #     raise NotImplementedError


    # async def change_flight_mode(self):
    #     await super().change_flight_mode()


    # async def failsafe(self):
    #     await super().failsafe()

    # async def idle(self, time):
    #     await asyncio.sleep(time)


    async def __print_status_text(self):
        try:
            async for status_text in self.drone.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return


    CONTEXT = "ROBOCIN_PILOT"
