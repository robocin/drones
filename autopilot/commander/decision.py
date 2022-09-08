import asyncio
from autopilot.constants import Constants, MessageType, Mission
from autopilot.commander.navigation import Navigation
from autopilot.debugger import STDOUT

class Decision:
    async def init(self, mission_type, drone):
        match(mission_type):
            case Mission.CALIBRATION:
                await self.calibration_pre_flight(drone)
                await self.calibration_mission(drone)
                await asyncio.sleep(10)
                await self.calibration_pos_flight(drone)
                return

            case Mission.SEARCH:
                await self.search_pre_flight(drone)
                await self.search_mission(drone)
                await self.search_pos_flight(drone)
                STDOUT.debug(self.CONTEXT,"Mission search ended")
                return

            case _:
                STDOUT.debug(MessageType.ERROR, "NO MISSION")
                return


    async def calibration_pre_flight(self, drone):
        await drone.action.set_takeoff_altitude(Constants.DRONE_SAFE_TAKEOFF_HEIGHT_METERS)
        await Navigation.arm(drone)


    async def calibration_mission(self, drone):
        await Navigation.takeoff(drone)


    async def calibration_pos_flight(self, drone):
        await Navigation.land(drone)
        return


    async def search_pre_flight(self, drone):
        STDOUT.debug(self.CONTEXT,"Search pre flight")
        await drone.action.set_takeoff_altitude(Constants.DRONE_TAKEOFF_HEIGHT_METERS)
        await Navigation.arm(drone)
        await Navigation.takeoff(drone)
        await asyncio.sleep(1)

    
    async def search_mission(self, drone):
        STDOUT.debug(self.CONTEXT,"Search mission")
        await Navigation.search(drone)
        

    async def search_pos_flight(self, drone):
        STDOUT.debug(self.CONTEXT,"Search pos flight")
        await Navigation.land(drone)
        exit(0)
    

    CONTEXT = "ROBOCIN_DECISION"