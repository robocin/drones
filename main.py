"""Lorrenzeti

This is the main module of the drone-autopilot project. 
Here the RobocinPilot is instantiated and the mission is started.
"""

import asyncio
from rcpilot.pilot import RobocinPilot
from rcpilot.environment.environment import Mission
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)


async def init():
    lorenzetti = RobocinPilot(Mission.THRUST)
    await lorenzetti.start_connection()
    await lorenzetti.start_mission()


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(init())
