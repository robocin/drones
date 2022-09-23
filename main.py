"""Lorrenzeti

This is the main module of the drone-autopilot project.
Here the RobocinPilot is instantiated and the mission is started.
"""

import asyncio
from rcpilot.modules.behavior.pilot import RobocinPilot


async def init():
    robocin_pilot = RobocinPilot()
    await robocin_pilot.connect_system()

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(init())
