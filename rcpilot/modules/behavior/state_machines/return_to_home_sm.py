"""Created by felipe-nunes on 23/09/2022
"""

from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType


class ReturnToHome(StateBase):

    _agent = None

    async def execute(self) -> None:
        Debug(MessageType.WARNING)("Returning to home.")
        return