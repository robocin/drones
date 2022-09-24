"""Created by felipe-nunes on 22/09/2022

    Drone autonomous agent representation class. 

    It is a asynchronous module and runs awaitable tasks.
"""

import asyncio
import warnings

from rcpilot.entities.drone.drone import Drone
from rcpilot.environment import Communication
from rcpilot.modules.behavior.state_machines.state_machine import StateMachine
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType
from rcpilot.utils.mission_type import MissionType
warnings.filterwarnings("ignore", category=DeprecationWarning) 

class RobocinPilot:
    _state_machine = StateMachine()
    _drone = Drone()
    _mission_type = MissionType.NO_MISSION

    def __init__(self, mission_type=MissionType.NO_MISSION):
        self._mission_type = mission_type

    async def connect_to_drone(self):
        if self._drone.is_connected:
            Debug(MessageType.WARNING)(f'Tried to connect but connection already established.')
        else:
            await self._drone.connect_system()


    CONTEXT = "ROBOCIN_PILOT"
