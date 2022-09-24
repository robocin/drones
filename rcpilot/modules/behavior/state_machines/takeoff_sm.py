"""Created by felipe-nunes on 23/09/2022
"""

from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.abstract_modules.state_base import StateBase


class TakeoffSM(StateBase):

    _context = None

    async def execute(self) -> None:
        print("executing takeoff")
        print("done takeoff")
        print("State machines ended")
