"""Created by felipe-nunes on 23/09/2022
"""

import asyncio
from rcpilot.entities.drone.odometry import Odometry
from rcpilot.entities.drone.telemetry import Telemetry
from sympy.geometry import Point3D
from rcpilot.packages.odometry_output import OdometryOutput

from rcpilot.packages.telemetry_output import TelemetryOutput


class Drone:
    _estimated_global_position = Point3D(0, 0, 0)
    _telemetry = Telemetry()
    _odometry = Odometry()
    # TODO: Create battery class
    _battery = None

    async def update(self):
        telemetry_output = await asyncio.create_task(self._odometry.execute())
        odometry_output = await asyncio.create_task(self._telemetry.execute())

        self.__update_global_position(telemetry_output, odometry_output)

    def __update_global_position(self, telemetry_output: TelemetryOutput, odometry_output: OdometryOutput):
        position_delta = Point3D(
            odometry_output.velocity.x, odometry_output.velocity.y, odometry_output.velocity.z)

        self._estimated_global_position = self._estimated_global_position + position_delta
