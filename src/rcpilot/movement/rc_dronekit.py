from dronekit import LocationGlobal, VehicleMode, connect
from pymavlink import mavutil
from rcpilot.logging.logger import Logger
from . import geometry
from enum import Enum


class ActionStatus(Enum):
    FINISHED = 0
    IN_PROGRESS = 1

logger = Logger.get_logger("movement", min_interval=0.5)
stopwatch = 0

class Drone:
    
    def __init__(self, connection_string, rate=100, is_gazebo=False):
        self.vehicle = connect(connection_string, rate)
        self.is_gazebo = is_gazebo
        self.pixhawk = None

    def condition_yaw(self, heading, direction, velocity=30, relative=True):
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            velocity,          # param 2, yaw speed deg/s
            direction,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
    
    def set_servo_position(self, servo_number, pwm_value):
        """Set PWM value for a specified servo."""
        message = self.vehicle.message_factory.command_long_encode(
            0, 0,        #target system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
            0,                              # confirmation
            servo_number,                   # param1 (servo number)
            pwm_value,                      # param2 (PWM value)
            0, 0, 0, 0, 0)                  # param3 ~ param7 (unused)
        
        self.vehicle.send_mavlink(message)
        
    def open_servo(self, servo_number=1):
        """Open a specified servo."""
        self.set_servo_position(servo_number,2000) # THESE VALUES HAVE TO BE TESTED BEFORE USING
        
    def close_servo(self, servo_number=1):
        """Close a specified servo."""
        self.set_servo_position(servo_number,1000) # THESE VALUES HAVE TO BE TESTED BEFORE USING

    def goto_relative_position(self, position, height=0):
        """
            Send goto relative position message to dronekit vehicle with the origin in the current vehicle position

            Parameters
            ----------
            vehicle : dronekit.Vehicle
                dronekit vehicle connection 
            position: Tuple(float, float)
                relative goto position in North East coordinate
            height: float = 0
                relative height, the default value stays in the same height
        """
        north, east = position

        logger.info(f"[COMMAND] going to relative position {position} (height: {height})")
        
        # convert NEU to FRD before sending message
        right = -east
        down = -height
        self._goto_position_target_body_frd(forward=north, right=right, down=down)


    def send_ned_velocity(self, velocity_x = 0, velocity_y = 0, velocity_z = 0):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        self.vehicle.send_mavlink(msg)

    def send_body_velocity(self, velocity_x = 0, velocity_y = 0, velocity_z = 0):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        self.vehicle.send_mavlink(msg)

    def goto_position(self, position, height=None, distance_threshold=0.2, only_check=False):  
        north, east = position
        # current relative to home altitude (NED)
        if height is None:
            down = self.vehicle.location.local_frame.down
        else:
            down = -height


        if not only_check:
            logger.info(f"[COMMAND] going to {position} (height: {height})")
            self._goto_position_target_local_ned(north, east, down)

        distance = geometry.distance_between_points(self.get_current_location_north_east(), position)
        height_diff = abs(self.vehicle.location.local_frame.down - down) >= distance_threshold

        if distance < distance_threshold and height_diff < distance_threshold:
            return ActionStatus.FINISHED
        
        return ActionStatus.IN_PROGRESS

    def takeoff(self, height, only_check=False):
        global stopwatch
        desired_altitude = height
        if not self.is_gazebo:
            cur_altitude = self.vehicle.location.global_relative_frame.alt
            desired_altitude = cur_altitude + height
        
        if not only_check:
            logger.info("[COMMAND] Taking off")
            self.vehicle.simple_takeoff(desired_altitude)

    def goto_relative_position(self, position, height=0):  
        """
            Send goto relative position message to dronekit vehicle with the origin in the current vehicle position

            Parameters
            ----------
            vehicle : dronekit.Vehicle
                dronekit vehicle connection 
            position: Tuple(float, float)
                relative goto position in North East coordinate
            height: float = 0
                relative height, the default value stays in the same height
        """
        north, east = position
        
        logger.info(f"[COMMAND] going to relative position {position} (height: {height})")
        
        # convert NED to FRD before sending message
        right = -east
        down = -height
        self._goto_position_target_body_frd(forward=north, right=right, down=down)


    def arm(self, only_check=False):
        if not only_check:
            logger.info("[COMMAND] Arming motors")
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True

        if self.vehicle.armed:
            self.vehicle.armed = True
            return ActionStatus.FINISHED
        
        return ActionStatus.IN_PROGRESS

    def land(self, only_check=False):
        if not only_check:
            logger.info("[COMMAND] Landing")
            self.vehicle.mode = VehicleMode("LAND")

        if not self.vehicle.armed:
                return ActionStatus.FINISHED
        
        return ActionStatus.IN_PROGRESS

    def get_current_location_north_east(self):
        n = self.vehicle.location.local_frame.north
        e = self.vehicle.location.local_frame.east
        return n, e
    
    def get_current_heading(self):
        return self.vehicle.heading

    def _goto_position_target_local_ned(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        logger.info(f"Sending goto command to N={north}, E={east}, D={down} using MAV_FRAME_LOCAL_NED")

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # Send command to the drone
        self.vehicle.send_mavlink(msg)
        
    def _goto_position_target_body_frd(self, forward, right, down=0):
        """
        Send SET_POSITION_TARGET_BODY_FRD command to request the vehicle fly to a specified
        location in the North, Right, Down frame.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            forward, right, down,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # Send command to the drone
        self.vehicle.send_mavlink(msg)

    def _goto_position_target_body_frd(self, forward, right, down=0):
        """
        Send SET_POSITION_TARGET_BODY_FRD command to request the vehicle fly to a specified
        location in the North, Right, Down frame.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            forward, right, down,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # Send command to the drone
        self.vehicle.send_mavlink(msg)
        
    def get_height(self):
        return self.vehicle.location.global_relative_frame.alt

    def set_home(self, latitude_deg=-35, longitude_deg=-8, altitude_m=7):
        logger.info("[COMMAND] Setting home position at", LocationGlobal(latitude_deg, longitude_deg, altitude_m))
        self.vehicle.home_location = LocationGlobal(latitude_deg, longitude_deg, altitude_m)

    def set_ekf_origin(self, latitude_deg=-35, longitude_deg=-8, altitude_m=7):
        latitude_deg = int(latitude_deg)
        longitude_deg = int(longitude_deg)
        altitude_m = int(altitude_m)

        logger.info(f"[COMMAND] Setting EKF Origin at lat: {latitude_deg}, long: {longitude_deg}, alt: {altitude_m}")
        self.vehicle.send_mavlink(self.vehicle.message_factory.set_gps_global_origin_encode(
            0,
            latitude_deg,
            longitude_deg,
            altitude_m
        ))
        
    def listen_mavlink_message(self, message_name, callback):
        self.vehicle.add_message_listener(message_name, callback)

    def remove_listener(self, message_name, callback):
        self.vehicle.remove_message_listener(message_name, callback)

    def get_camera_params(camera):
        # New cameras should be add here
        ## Translation
        # 0 = horizontal fov
        # 1 = vertical fov
        # 2 = horizontal resolution
        # 3 = vertical resolution
        params = dict()
        if camera == 'realsense':
            params["horizontal_fov"] = 70 * 3.14 / 180
            params["vertical_fov"] = 43 * 3.14 / 180
            params["horizontal_resolution"] = 800
            params["vertical_resolution"] = 600
            
        elif camera == 'quadrada':
            params["horizontal_fov"] = 120 * 3.14 / 180
            params["vertical_fov"] = 90 * 3.14 / 180
            params["horizontal_resolution"] = 640
            params["vertical_resolution"] = 480
            
        elif camera == 'c920':
            params["horizontal_fov"] = 78 * 3.14 / 180
            params["vertical_fov"] = 49 * 3.14 / 180
            params["horizontal_resolution"] = 640
            params["vertical_resolution"] = 480
        else: 
            logger.info("[WARNING] Using default camera params")
            params["horizontal_fov"] = 118.2 * 3.14/180
            params["vertical_fov"] = 69.5 * 3.14/180
            params["horizontal_resolution"] = 1280
            params["vertical_resolution"] = 720
        return params

    def _send_precision_land_message(self, x_angle, y_angle, distance):
        msg = self.vehicle.message_factory.landing_target_encode(
            0,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            x_angle,
            y_angle,
            distance,  # distance
            0,  # size_x
            0,  # size_y
            0,  # x
            0,  # y
            0,  # z
            (1, 0, 0, 0),  # q,
            0,  # type
            0,  # position_valid
        )

        self.vehicle.send_mavlink(msg)

        self.vehicle.flush()


    def precision_land(self, camera_params, center_vector):
        """
            Send precision land message to dronekit vehicle corresponding to center vector  

            Parameters
            ----------
            vehicle : dronekit.Vehicle
                dronekit vehicle connection 
            camera_params: dict
                camera parameters dictionary (horizontal fov, vertical fov, horizontal resolution and vertical resolution) 
                
            Returns 
            -------
            Returns True if the center vector was not None, otherwise False
            
            Bool
        """
        if center_vector is None:
            return False
        x, y = center_vector

        x_angle = float(x*camera_params["horizontal_fov"]/camera_params["horizontal_resolution"])
        y_angle = float(y*camera_params["vertical_fov"]/camera_params["vertical_resolution"])

        self._send_precision_land_message(x_angle, y_angle, self.vehicle.location.global_relative_frame.alt)

        if(self.vehicle.mode != "LAND"):
            self.vehicle.mode = "LAND"

        return True
