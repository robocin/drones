"""
    Navigation module

    Feeds from decision and actuates on drone for movimentation.
"""

import asyncio
from autopilot.debugger import STDOUT
from autopilot.constants import Constants
from autopilot.constants import MessageType
from mavsdk.offboard import PositionNedYaw, OffboardError, Attitude


class Navigation:
    @classmethod
    async def arm(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Arming")
        await drone.action.arm()
        STDOUT.debug(cls.CONTEXT, "Drone armed")
        STDOUT.debug(MessageType.WARNING, "Step back motor starting")

    @classmethod
    async def takeoff(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Taking off")
        await drone.action.takeoff()

    @classmethod
    async def land(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Landing")
        await drone.action.land()

    @classmethod
    async def start_offboard_with_ned(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        STDOUT.debug(cls.CONTEXT, "Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            STDOUT.debug(MessageType.ERROR, "Starting offboard mode failed with error code: {}".format(
                error._result.result))
            STDOUT.debug(MessageType.WARNING, "Disarming")
            await drone.action.disarm()
            return

    @classmethod
    async def search_square(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        STDOUT.debug(cls.CONTEXT, "Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            STDOUT.debug(MessageType.ERROR, "Starting offboard mode failed with error code: {}".format(
                error._result.result))
            STDOUT.debug(MessageType.WARNING, "Disarming")
            await drone.action.disarm()
            return

        STDOUT.debug(cls.CONTEXT, "Head north")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 0.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Go to north")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 0.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Head east")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Go to east")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, 90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Head south")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, 180.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Go to south")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, 180.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Head west")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, -90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Go to west")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, -90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Head north")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 0.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        STDOUT.debug(cls.CONTEXT, "Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            STDOUT.debug(
                MessageType.ERROR, f"Stopping offboard mode failed with error code: {error._result.result}.")

    @classmethod
    async def disarm(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Disarming")
        await drone.action.disarm()

    @classmethod
    async def set_thrust(cls, drone, thrust_level):
        STDOUT.debug(cls.CONTEXT, "Set thrust to {}".format(thrust_level))
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust_level))

    CONTEXT = "ROBOCIN_NAVIGATION"
