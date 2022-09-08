import asyncio
from pilot import RobocinPilot
from constants import Mission

async def init():
    lorenzetti = RobocinPilot(Mission.CALIBRATION)
    await lorenzetti.start_connection()
    await lorenzetti.arm()
    

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(init())
