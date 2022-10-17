"""Lorrenzeti

This is the main module of the drone-autopilot project.
Here the RobocinPilot is instantiated and the mission is started.
"""

import asyncio
from rcpilot.modules.behavior.pilot import RobocinPilot
from rcpilot.utils.mission_type import MissionType
from rcpilot.utils.connection_type import ConnectionType


async def init():
    robocin_pilot = RobocinPilot(MissionType.TESTING, ConnectionType.HARDWARE)
    await robocin_pilot.connect_to_drone()
    await robocin_pilot.execute()

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(init())
