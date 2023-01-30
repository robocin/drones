
"""Lorrenzeti

This is the main module of the drone-autopilot project.
Here the RobocinPilot is instantiated and the mission is started.
"""

from rcpilot.modules.vision import Vision
from rcpilot.modules.behavior.pilot import RobocinPilot
from rcpilot.utils.mission_type import MissionType
from rcpilot.utils.connection_type import ConnectionType

import time
import asyncio
from dronekit import connect, VehicleMode
import dronekit_sitl

def connect_to_simulation():
    simulation = dronekit_sitl.start_default()
    connection_string = simulation.connection_string()


def connect_to_drone(connection_string=None):
    connection_string = connection_string if connection_string is not None else '/dev/ttyACM0'
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle


def arm_vehicle(vehicle):
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)


def takeoff(vehicle, altitude=10):
    vehicle.simple_takeoff(altitude)
    while vehicle.location.global_relative_frame.alt < altitude:
        time.sleep(1)
    return


async def init():
    #robocin_pilot = RobocinPilot()
    #vision = Vision()
    #await vision.execute()
    #await robocin_pilot.connect_to_drone()
    #await robocin_pilot.execute()
    
    #######
    
    #robocin_pilot = RobocinPilot(MissionType.TESTING, ConnectionType.HARDWARE)
    #await robocin_pilot.connect_to_drone()
    #await robocin_pilot.execute()

# VISION
"""
if __name__ == '__main__':
    #loop = asyncio.get_event_loop()
    #loop.run_until_complete(init())
    vision = Vision()
    while True:
        begin = time.time()
        vision.execute()
        vision._share_package()
        end = time.time()
        print(end - begin)
"""

# DRONEKIT TEST
"""
if __name__ == "__main__":
    # Connect to simulation
    connect_to_simulation()

    # Connect to drone
    vehicle = connect_to_drone()

    # Arm vehicle
    arm_vehicle(vehicle)

    # Takeoff
    takeoff(vehicle)

    # Land
    vehicle.mode = VehicleMode('LAND')

    # Close connection
    vehicle.close()

    # Shut down simulator
    dronekit_sitl.stop_default()
"""