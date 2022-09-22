"""Created by felipe-nunes on 22/09/2022

    Drone representation class. 
    
    Has vision, decision and navigation modules that together
    commands the drone. 

    It is a asynchronous module and runs awaitable tasks.
"""

import asyncio
from mavsdk import System
from abc import ABC
from rcpilot.modules.decision import Decision
from rcpilot.modules.navigation import Navigation
from rcpilot.modules.vision import Vision
from rcpilot.environment import Arena, Communication, Navigation, Takeoff, Land
from rcpilot.environment.environment import MessageType
from rcpilot.environment.environment import Mission
from rcpilot.environment.environment import Constants
from rcpilot.utils.debugger import Debug


class SafePilot(ABC):
    def __init__(self, mission_type: Mission) -> None:
        self.mission_type = mission_type

    def __ensure_safe_takeoff_height():
        raise NotImplementedError

    CONTEXT = "SAFE_PILOT"


class Drone(SafePilot):
    def __init__(self, mission_type: Mission) -> None:
        super().__init__(mission_type)
        self.system: System = System()
        self.vision: Vision = Vision()
        self.decision: Decision = Decision()

    async def start_connection(self):
        Debug(self.CONTEXT)("Connecting to {}".format(
            Communication.CONN_STRING))
        await self.system.connect(system_address=Communication.CONN_STRING)
        self.thread_listener = asyncio.ensure_future(
            self.__print_status_text())

        Debug(self.CONTEXT)("Waiting for drone to connect")
        async for state in self.system.core.connection_state():
            if state.is_connected:
                Debug(self.CONTEXT)("Connected to drone!")
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

    CONTEXT = "DRONE"
