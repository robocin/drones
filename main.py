"""Lorrenzeti

This is the main module of the drone-autopilot project.
Here the RobocinPilot is instantiated and the mission is started.
"""

import asyncio
#from rcpilot.modules.behavior.pilot import RobocinPilot
from rcpilot.modules.vision import Vision


async def init():
    #robocin_pilot = RobocinPilot()
    vision = Vision()
    await vision.execute()
    #await robocin_pilot.connect_to_drone()
    #await robocin_pilot.execute()

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(init())
    
    
