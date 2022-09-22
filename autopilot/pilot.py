"""
    Drone representation class. 
    
    Has vision, decision and navigation modules that together
    commands the drone. 

    It is a asynchronous module and runs awaitable tasks.
"""

import asyncio
from mavsdk import System
from abc import ABC
from autopilot.commander.decision import Decision
from autopilot.commander.navigation import Navigation
from autopilot.commander.vision import Vision
from autopilot.constants import MessageType
from autopilot.constants import Mission
from autopilot.constants import Constants
from autopilot.debugger import STDOUT


class SafePilot(ABC):
    def __init__(self, mission_type: Mission) -> None:
        self.mission_type = mission_type

    def __ensure_safe_takeoff_height():
        raise NotImplementedError

    CONTEXT = "PILOT"


class RobocinPilot(SafePilot):
    def __init__(self, mission_type: Mission) -> None:
        super().__init__(mission_type)
        self.system: System = System()
        self.vision: Vision = Vision()
        self.decision: Decision = Decision()

    async def start_connection(self):
        STDOUT.debug(self.CONTEXT, "Connecting to {}".format(
            Constants.COMM_CONN_STRING))
        await self.system.connect(system_address=Constants.COMM_CONN_STRING)
        self.thread_listener = asyncio.ensure_future(
            self.__print_status_text())

        STDOUT.debug(self.CONTEXT, "Waiting for drone to connect")
        async for state in self.system.core.connection_state():
            if state.is_connected:
                STDOUT.debug(self.CONTEXT, "Connected to drone!")
                break

    async def start_mission(self):
        await self.decision.init(self.mission_type, self.system)
        return

    async def __print_status_text(self):
        try:
            async for status_text in self.drone.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return

    CONTEXT = "ROBOCIN_PILOT"
