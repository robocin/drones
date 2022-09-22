"""Created by felipe-nunes on 22/09/2022
"""

import asyncio
from rcpilot.utils.debugger import Debug
from rcpilot.environment.environment import Constants
from rcpilot.environment.environment import MessageType
from mavsdk.offboard import PositionNedYaw, OffboardError, Attitude


class Navigation:
    @classmethod
    async def arm(cls, drone):
        Debug(cls.CONTEXT)("Arming")
        await drone.action.arm()
        Debug(cls.CONTEXT)("Drone armed")
        Debug(MessageType.WARNING)("Step back motor starting")

    @classmethod
    async def takeoff(cls, drone):
        Debug(cls.CONTEXT)("Taking off")
        await drone.action.takeoff()

    @classmethod
    async def land(cls, drone):
        Debug(cls.CONTEXT)("Landing")
        await drone.action.land()

    @classmethod
    async def start_offboard_with_ned(cls, drone):
        Debug(cls.CONTEXT)("Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        Debug(cls.CONTEXT)("Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            Debug(MessageType.ERROR)("Starting offboard mode failed with error code: {}".format(error._result.result))
            Debug(MessageType.WARNING)("Disarming")
            await drone.action.disarm()
            return

    @classmethod
    async def search_square(cls, drone):
        Debug(cls.CONTEXT)("Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        Debug(cls.CONTEXT)("Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            Debug(MessageType.ERROR)("Starting offboard mode failed with error code: {}".format(error._result.result))
            Debug(MessageType.WARNING)("Disarming")
            await drone.action.disarm()
            return

        Debug(cls.CONTEXT)("Head north")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 0.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Go to north")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 0.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Head east")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Go to east")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, 90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Head south")
        await drone.offboard.set_position_ned(PositionNedYaw(Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, 180.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Go to south")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS, Constants.DRONE_SEARCH_NED_HEIGHT, 180.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Head west")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, Constants.DRONE_SEARCH_SQUARE_EDGE_METERS , Constants.DRONE_SEARCH_NED_HEIGHT, -90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Go to west")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, -90.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)

        Debug(cls.CONTEXT)("Head north")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, Constants.DRONE_SEARCH_NED_HEIGHT, 0.0))
        await asyncio.sleep(Constants.DRONE_SEARCH_IDLE_TIME_S)


        Debug(cls.CONTEXT)("Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            Debug(MessageType.ERROR)(f"Stopping offboard mode failed with error code: {error._result.result}.")


    @classmethod
    async def disarm(cls, drone):
        Debug(cls.CONTEXT)("Disarming")
        await drone.action.disarm()

    @classmethod
    async def set_thrust(cls, drone, thrust_level):
        Debug(cls.CONTEXT)(f'Set thrust to {thrust_level}')
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust_level))

    CONTEXT = "ROBOCIN_NAVIGATION"
