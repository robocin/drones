from abc import ABC, abstractmethod
from rcpilot.abstract_modules.singleton_meta import SingletonMeta
from rcpilot.packages.telemetry_output import TelemetryOutput


class TelemetryBase(ABC, SingletonMeta):
    @abstractmethod
    async def _get_package(self) -> TelemetryOutput:
        pass

    @abstractmethod
    async def execute(self):
        pass
