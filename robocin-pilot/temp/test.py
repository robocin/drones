from definitions import connect_to_vehicle, arm_and_takeoff, goto, set_velocity_for_duration, set_yaw, get_position
import time

vehicle = connect_to_vehicle()
# set_yaw(vehicle, 0)
# arm_and_takeoff(vehicle, 1)
goto(vehicle, 0, 0, -1, frame="local")
# time.sleep(3)

# goto(vehicle, 1, 0, 0, frame="test")
# goto(vehicle, -1, 0, 0, frame="test")
# for i in range(0, 5):
#     goto(vehicle, 0, 1, 0, frame="body")
#     goto(vehicle, 0, -1, 0, frame="body")

# goto(vehicle, 0, 0, 0, frame="test")

# for i in range(1):
#     # while (True):
#     goto(vehicle, 1, 0, -1, frame="local")
#     goto(vehicle, 1, 1, -1, frame="local")
#     goto(vehicle, 0, 1, -1, frame="local")
#     goto(vehicle, 0, 0, -1, frame="local")

# for i in range(5):
#     goto(vehicle, 1, 0, 0)
#     goto(vehicle, 0, 1, 0)
#     goto(vehicle, -1, 0, 0)
#     goto(vehicle, 0, -1, 0)


# time.sleep(3)
# print(get_position(vehicle))
# print(get_position(vehicle))

# goto(vehicle, 2, 0, -1, frame="local")
# time.sleep(3)
# print(" Local Location: %s" % vehicle.location.local_frame)
# goto(vehicle, 0, 0, -1, frame="local")
# print(" Local Location: %s" % vehicle.location.local_frame)
# time.sleep(3)
# print(" Local Location: %s" % vehicle.location.local_frame)

# set_yaw(vehicle, 0)

# goto(vehicle, 4, 0, 0)
# time.sleep(3)
# goto(vehicle, 0, 4, 0)
# time.sleep(3)
# goto(vehicle, -4, 0, 0)
# time.sleep(3)
# goto(vehicle, 0, -4, 0)

# set_velocity_for_duration(vehicle, 1, 0, 0, 5)
