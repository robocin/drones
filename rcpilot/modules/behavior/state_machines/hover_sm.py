"""Created by felipe-nunes on 22/09/2022
"""


import asyncio

from rcpilot.utils.debugger import Debug
from rcpilot.utils.message_type import MessageType


class HoverSM:

    _agent = None
    _seconds = 0

    def __init__(self, seconds) -> None:
        self._seconds = seconds

    async def execute(self) -> None:
        Debug(MessageType.INFO)(f"Hovering for {self._seconds} seconds.")
        await asyncio.sleep(self._seconds)
