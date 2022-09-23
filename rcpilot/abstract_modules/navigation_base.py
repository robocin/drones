"""Created by felipe-nunes on 22/09/2022
"""

from abc import ABC, abstractmethod

from rcpilot.abstract_modules.singleton_meta import SingletonMeta


class NavigationBase(ABC, SingletonMeta):
    @abstractmethod
    async def _get_package(self):
        pass

    @abstractmethod
    async def execute(self):
        pass
