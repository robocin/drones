"""Created by felipe-nunes on 23/09/2022
"""

from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.modules.behavior.state_machines.takeoff_sm import TakeoffSM
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType


class PreflightSM(StateBase):

    _agent = None

    async def execute(self) -> None:
        Debug(MessageType.LOG)("Arming.")
        await self.agent.system.action.arm()
        self.agent.transition_to(TakeoffSM())