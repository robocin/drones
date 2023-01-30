"""Created by felipe-nunes on 22/09/2022
"""

from abc import ABC, abstractmethod
from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.packages.decision_output import DecisionOutput


class DecisionBase(ABC, SingletonMeta):
    @abstractmethod
    async def _share_package(self) -> DecisionOutput:
        pass

    @abstractmethod
    async def execute(self):
        pass
