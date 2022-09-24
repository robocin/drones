"""Created by felipe-nunes on 23/09/2022
"""

from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.modules.behavior.state_machines.return_to_home_sm import ReturnToHome
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType


class TakeoffSM(StateBase):

    _agent = None

    async def execute(self) -> None:
        Debug(MessageType.LOG)("Taking off.")
        await self.agent.system.action.takeoff()
        self.agent.transition_to(ReturnToHome())
        
