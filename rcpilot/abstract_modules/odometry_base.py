from abc import ABC, abstractmethod
from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.packages.odometry_output import OdometryOutput


class OdometryBase(ABC, SingletonMeta):
    @abstractmethod
    async def _get_package(self) -> OdometryOutput:
        pass

    @abstractmethod
    async def execute(self):
        pass
