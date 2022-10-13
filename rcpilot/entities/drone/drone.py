"""Created by felipe-nunes on 23/09/2022
"""

import asyncio
from mavsdk import System
from rcpilot.entities.drone.odometry import Odometry
from rcpilot.entities.drone.telemetry import Telemetry
from sympy.geometry import Point3D
from rcpilot.environment import Communication
from rcpilot.packages.odometry_output import OdometryOutput
from rcpilot.packages.telemetry_output import TelemetryOutput
from rcpilot.utils.debugger import Debug
from rcpilot.utils.connection_type import ConnectionType
from rcpilot.utils.enable_mavlink_connection import enable_mavlink_connection


class Drone:
    _estimated_global_position = Point3D(0, 0, 0)
    _system: System = System()
    _telemetry = Telemetry()
    _odometry = Odometry()
    _connection_state = None
    # TODO: Create battery class
    _battery = None

    @property
    def is_connected(self):
        return self._connection_state is not None

    async def connect_system(self, connection_type):
        self.init_connection_port(connection_type)

        connection_string = self.resolve_connection_string(connection_type)

        Debug(self.CONTEXT)(f'Connecting to {connection_string}.')
        await self._system.connect(connection_string)

        async for state in self._system.core.connection_state():
            if state.is_connected:
                self._connection_state = state
                Debug(self.CONTEXT)(f'Connected to system.')
                break

    def init_connection_port(self, connection_type):
        if(connection_type == ConnectionType.HARDWARE):
            enable_mavlink_connection()

    def resolve_connection_string(self, connection_type):
        # BEWARE: when defining a new connection type, first a new enum must be defined
        #         in environment.py > class Communication, then, a new corresponding
        #         if statement must be declared here

        if(connection_type == ConnectionType.SIMULATION):
            return Communication.SIMULATION_CONN_STRING
        elif(connection_type == ConnectionType.HARDWARE):
            return Communication.HARDWARE_CONN_STRING

    async def update(self):
        telemetry_output = await asyncio.create_task(self._odometry.execute())
        odometry_output = await asyncio.create_task(self._telemetry.execute())

        self.__update_global_position(telemetry_output, odometry_output)

    def __update_global_position(self, telemetry_output: TelemetryOutput, odometry_output: OdometryOutput):
        position_delta = Point3D(
            odometry_output.velocity.x, odometry_output.velocity.y, odometry_output.velocity.z)

        self._estimated_global_position = self._estimated_global_position + position_delta

    CONTEXT = "DRONE"