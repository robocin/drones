"""Created by felipe-nunes on 23/09/2022
"""

from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.abstract_modules.state_base import StateBase
from rcpilot.modules.behavior.state_machines.takeoff_sm import TakeoffSM


class PreflightSM(StateBase):

    _context = None

    async def execute(self) -> None:
        print("executing preflight")
        print("done preflight")
        self._context.transition_to(TakeoffSM())