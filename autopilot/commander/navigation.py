import asyncio
from autopilot.debugger import STDOUT
from autopilot.constants import Constants
from autopilot.constants import MessageType
from mavsdk.offboard import PositionNedYaw, OffboardError

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
    async def search(cls, drone):
        STDOUT.debug(cls.CONTEXT,"Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        STDOUT.debug(cls.CONTEXT,"Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            STDOUT.debug(MessageType.ERROR,"Starting offboard mode failed with error code: {}".format(error._result.result))
            STDOUT.debug(MessageType.WARNING, "Disarming")
            await drone.action.disarm()
            return

        STDOUT.debug(cls.CONTEXT,"Head north")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Go to north")
        await drone.offboard.set_position_ned(PositionNedYaw(2.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Head east")
        await drone.offboard.set_position_ned(PositionNedYaw(2.0, 0.0, -2.0, 90.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Go to east")
        await drone.offboard.set_position_ned(PositionNedYaw(2.0, 2.0, -2.0, 90.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Head south")
        await drone.offboard.set_position_ned(PositionNedYaw(2.0, 2.0, -2.0, 180.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Go to south")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 2.0, -2.0, 180.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Head west")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 2.0, -2.0, -90.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Go to west")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, -90.0))
        await asyncio.sleep(2)

        STDOUT.debug(cls.CONTEXT,"Head north")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(2)


        STDOUT.debug(cls.CONTEXT, "Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            STDOUT.debug(MessageType.ERROR, f"Stopping offboard mode failed with error code: {error._result.result}.")

    
    @classmethod
    async def disarm(cls, drone):
        STDOUT.debug(cls.CONTEXT, "Disarming")
        await drone.action.disarm()

    
    CONTEXT = "ROBOCIN_NAVIGATION"