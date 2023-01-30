"""Created by felipe-nunes on 23/09/2022
"""

import asyncio
from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.modules.behavior.state_machines.hover_sm import HoverSM
from rcpilot.modules.behavior.state_machines.return_to_home_sm import ReturnToHomeSM
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType
from rcpilot.utils.mission_type import MissionType


class TakeoffSM(StateBase):

    _agent = None

    async def execute(self) -> None:
        Debug(MessageType.INFO)("Taking off.")

        await self.agent.system.action.takeoff()

        await asyncio.sleep(2)

        if self.agent.mission_type == MissionType.TESTING:
            self.agent.transition_to(HoverSM(seconds=10))
        else:
            self.agent.transition_to(ReturnToHomeSM())
        
