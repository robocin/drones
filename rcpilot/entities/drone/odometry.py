"""Created by felipe-nunes on 23/09/2022
"""


from rcpilot.abstract_modules.odometry_base import OdometryBase
from rcpilot.packages.odometry_output import OdometryOutput


class Odometry(OdometryBase):
    _odometry_output = OdometryOutput()

    def __init__(self):
        self._mavsdk_odometry = self.__get_mavsdk_odometry()

    def __get_mavsdk_odometry(self):
        pass

    async def _get_package(self) -> OdometryOutput:
        return self._odometry_output

    async def execute(self):
        # Collect odometry from mavsdk
        self.__get_mavsdk_odometry()

        return self._odometry_output
