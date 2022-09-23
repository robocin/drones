"""Created by felipe-nunes on 22/09/2022

    Drone autonomous agent representation class. 

    It is a asynchronous module and runs awaitable tasks.
"""

import asyncio
from mavsdk import System
from rcpilot.entities.drone.drone import Drone
from rcpilot.environment import Communication
from rcpilot.modules.behavior.state_machines.state_machine import StateMachine
from rcpilot.utils.debugger import Debug
from rcpilot.utils.mission_type import MissionType


class RobocinPilot:
    _state_machine = StateMachine()
    _drone = Drone()
    _system: System = System()
    _mission_type = MissionType.NO_MISSION

    def __init__(self, mission_type=MissionType.NO_MISSION):
        self._mission_type = mission_type

    async def connect_system(self):
        Debug(self.CONTEXT)(f'Connecting to {Communication.CONN_STRING}')
        await self._system.connect(system_address=Communication.CONN_STRING)

        Debug(self.CONTEXT)(f'Waiting for system to connect')
        async for state in self._system.core.connection_state():
            if state.is_connected:
                Debug(self.CONTEXT)(f'Connected to system!')
                break

    CONTEXT = "ROBOCIN_PILOT"
