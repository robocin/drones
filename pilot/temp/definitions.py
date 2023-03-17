from dronekit import LocationGlobalRelative, mavutil, VehicleMode
import time

from dronekit import connect, VehicleMode

# vehicle = dronekit.connect("127.0.0.1:14551", wait_ready=True)


def connect_to_vehicle(connection_string="127.0.0.1:14551"):
    print("==> Connecting to vehicle firmware on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    print("Connected.")
    return vehicle


def arm_and_takeoff(vehicle, targetAltitude, block=True):
    print("==> Commencing Arm and Takeoff")

    print("...Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("...Changing to GUIDED mode")
    vehicle.mode = VehicleMode("GUIDED")

    print("...Arming motors")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(targetAltitude)

    if (block):
        print("...waiting for targetAltitude to be reached")
        while True:
            if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
                print("targetAltitude reached")
                break
            time.sleep(1)


def land_and_disarm():
    pass


def set_velocity_for_duration(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def goto(vehicle, north, east, down, frame="body"):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.

    @param frame: "body" or "local"
    """
    initial_pos = get_position(vehicle)

    if (frame == "local"):
        mavlink_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    elif (frame == "body"):
        mavlink_frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
    elif (frame == "test"):
        mavlink_frame = mavutil.mavlink.MAV_FRAME_BODY_NED

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavlink_frame,  # frame
        # nao sei de onde veio # 0b100111000111,  # type_mask (positions and yaw enabled)
        # eh o que tava antes, funcionando # 0b0000111111111000,  # type_mask (only positions enabled)
        0b110111111000,
        north, east, down,
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)

    print("...waiting for target position to be reached")
    while True:

        if (frame == "local"):
            if [north, east, down] == get_position(vehicle):
                print("target position reached")
                break
        elif (frame == "body"):
            displacement = []
            for i, j in zip(get_position(vehicle), initial_pos):
                # displacement.append(i - j)
                displacement.append(j-i)
            print("initial_pos: ", initial_pos)
            print("displacement: ", displacement)
            print("[north, east, down]: ", [north, east, down])
            print()
            print()
            print()
            if [north, east, down] == displacement:
                print("target position reached")
                break

        time.sleep(1)

    time.sleep(1)


def set_yaw(vehicle, heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def get_position(vehicle, digitsPrecision=0):

    north = vehicle.location.local_frame.north
    east = vehicle.location.local_frame.east
    down = vehicle.location.local_frame.down

    return [
        round(north, digitsPrecision),
        round(east, digitsPrecision),
        round(down, digitsPrecision)
    ]
