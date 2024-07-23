import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong

class SetGPSGlobalOriginNode(Node):
    def __init__(self):
        super().__init__('set_gps_global_origin_node')
        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.request = CommandLong.Request()

    def send_request(self, latitude, longitude, altitude):
        self.request.command = 48  # MAVLink command id for SET_GPS_GLOBAL_ORIGIN
        self.request.param1 = latitude
        self.request.param2 = longitude
        self.request.param3 = altitude
        self.request.param4 = 0.0  # change this line
        self.request.param5 = 0.0  # and this line
        self.request.param6 = 0.0  # and this line
        self.request.param7 = 0.0  # and this line
        self.request.confirmation = False
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print("Successfully set GPS global origin.")
        else:
            print("Failed to set GPS global origin.")


def main(args=None):
    rclpy.init(args=args)
    node = SetGPSGlobalOriginNode()
    latitude = -8.052240  # replace with your desired latitude
    longitude = -34.928612  # replace with your desired longitude
    altitude = 0.0  # replace with your desired altitude
    node.send_request(latitude, longitude, altitude)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
