from dronekit import connect, VehicleMode
import dronekit_sitl
import time


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
