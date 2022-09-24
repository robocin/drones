"""Created by felipe-nunes on 23/09/2022
"""


from rcpilot.abstract_modules.telemetry_base import TelemetryBase
from rcpilot.packages.telemetry_output import TelemetryOutput


class Telemetry(TelemetryBase):
    _telemetry_output = TelemetryOutput()

    def __init__(self):
        self._mavsdk_telemetry = self.__get_mavsdk_telemetry()

    def __get_mavsdk_telemetry(self):
        pass

    async def _share_package(self) -> TelemetryOutput:
        return self._telemetry_output

    async def execute(self):
        # Collect telemetry from mavlink
        self.__get_mavsdk_telemetry()

        return self._telemetry_output
