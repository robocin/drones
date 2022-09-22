"""Created by felipe-nunes on 22/09/2022
"""

import asyncio
from rcpilot.environment.environment import Constants, MessageType, Mission
from rcpilot.modules.navigation import Navigation
from rcpilot.utils.debugger import Debug

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
                Debug(self.CONTEXT)("Mission search ended")
                return

            case Mission.THRUST:
                await self.thurst_pre_flight(drone)
                await self.thrust_mission(drone)
                await self.thrust_pos_flight(drone)
                return

            case _:
                Debug(MessageType.ERROR)("NO MISSION")
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
        Debug(self.CONTEXT)("Search pre flight")
        await drone.action.set_takeoff_altitude(Constants.DRONE_TAKEOFF_HEIGHT_METERS)
        await Navigation.arm(drone)
        await Navigation.takeoff(drone)
        await asyncio.sleep(5)

    async def search_mission(self, drone):
        Debug(self.CONTEXT)("Search mission")
        await Navigation.search_square(drone)

    async def search_pos_flight(self, drone):
        Debug(self.CONTEXT)("Search pos flight")
        await Navigation.land(drone)
        exit(0)

    async def thurst_pre_flight(self, drone):
        Debug(self.CONTEXT)("Thrust pre flight")
        await drone.action.set_takeoff_altitude(Constants.DRONE_TAKEOFF_HEIGHT_METERS)
        await Navigation.arm(drone)

    async def thrust_mission(self, drone):
        Debug(self.CONTEXT)("Thrust mission")
        await Navigation.start_offboard_with_ned(drone)
        await Navigation.set_thrust(drone, thrust_level=0.2)
        await asyncio.sleep(5)
        await Navigation.set_thrust(drone, thrust_level=0.4)
        await asyncio.sleep(5)
        await Navigation.set_thrust(drone, thrust_level=0.6)
        await asyncio.sleep(5)
        await Navigation.set_thrust(drone, thrust_level=0.7)
        await asyncio.sleep(5)
        await Navigation.set_thrust(drone, thrust_level=0.8)
        await asyncio.sleep(5)
        await Navigation.set_thrust(drone, thrust_level=0.9)
        await asyncio.sleep(5)
        await Navigation.set_thrust(drone, thrust_level=1.0)
        await asyncio.sleep(5)

    async def thrust_pos_flight(self, drone):
        Debug(self.CONTEXT)("Search pos flight")
        await Navigation.land(drone)
        exit(0)

    CONTEXT = "ROBOCIN_DECISION"
