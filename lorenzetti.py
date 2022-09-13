import asyncio
from autopilot.pilot import RobocinPilot
from autopilot.constants import Mission
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning) 


async def init():
    lorenzetti = RobocinPilot(Mission.THRUST)
    await lorenzetti.start_connection()
    await lorenzetti.start_mission()
    

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(init())
