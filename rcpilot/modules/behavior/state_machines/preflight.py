"""Created by felipe-nunes on 23/09/2022
"""

from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.modules.behavior.state_machines.takeoff_sm import TakeoffSM
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType
from rcpilot.utils.mission_type import MissionType


class PreflightSM(StateBase):

    _agent = None

    async def execute(self) -> None:
        Debug(MessageType.INFO)("Arming.")
        await self.agent.system.action.arm()

        if self.agent.mission_type == MissionType.TESTING:
            Debug(MessageType.WARNING)(f'Mission type is TESTING')
            self.agent.transition_to(TakeoffSM())
        else:
            self.agent.transition_to()
            await self.agent.system.action.disarm()