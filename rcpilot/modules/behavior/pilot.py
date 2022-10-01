"""Created by felipe-nunes on 22/09/2022

    Drone autonomous agent representation class. 

    It is a asynchronous module and runs awaitable tasks.
"""

import asyncio
import warnings
from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.abstract_modules.vision_base import VisionBase

from rcpilot.entities.drone.drone import Drone
from rcpilot.environment import Communication
from rcpilot.modules.behavior.state_machines.preflight import PreflightSM
# from rcpilot.modules.decision import Decision
# from rcpilot.modules.vision import Vision
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType
from rcpilot.utils.mission_type import MissionType
warnings.filterwarnings("ignore", category=DeprecationWarning) 

class RobocinPilot(SingletonMeta):
    _state = None
    _drone = Drone()
    # _vision = Vision()
    # _decision = Decision()
    _mission_type = MissionType.NO_MISSION
    __enable_execution = True

    def __init__(self, mission_type=MissionType.NO_MISSION):
        self._mission_type = mission_type
        self.transition_to(PreflightSM())

    @property
    def system(self):
        return self._drone._system

    @property
    def mission_type(self):
        return self._mission_type

    def transition_to(self, state):
        Debug(self.CONTEXT)(f"Transition to {type(state).__name__}")
        self._state = state
        self._state.agent = self

    async def connect_to_drone(self):
        if self._drone.is_connected:
            Debug(MessageType.WARNING)(f'Tried to connect but connection already established.')
        else:
            await self._drone.connect_system()

    async def execute(self):
        while self.__enable_execution:
            if "ReturnToHomeSM" == type(self._state).__name__:
                break
            await self._state.execute()

    CONTEXT = "ROBOCIN_PILOT"
