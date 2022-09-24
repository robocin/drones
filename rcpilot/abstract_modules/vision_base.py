"""Created by felipe-nunes on 22/09/2022
"""

from abc import ABC, abstractmethod
from rcpilot.abstract_modules.singleton_meta import SingletonMeta


class VisionBase(ABC, SingletonMeta):
    @abstractmethod
    async def _share_package(self):
        pass

    @abstractmethod
    async def execute(self):
        pass
